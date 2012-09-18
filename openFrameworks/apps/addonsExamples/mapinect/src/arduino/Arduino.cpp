#include "Arduino.h"

#include "Feature.h"
#include "ofxXmlSettings.h"
#include <direct.h> // for getcwd
#include "Plane3D.h"
#include "utils.h"
#include "Globals.h"
#include "Constants.h"

#include "transformationUtils.h"

namespace mapinect {

#define		ARDUINO_CONFIG		"ArduinoConfig:"

#define		ANGLE_UNDEFINED		MAXCHAR
#define		ANGLE_DEFAULT		0
#define		KEY_UNDEFINED		""

#define		MOTOR_ANGLE_OFFSET_DEFAULT	128

	static string	COM_PORT;
	static char		KEY_MOVE_1R;
	static char		KEY_MOVE_1L;
	static char		KEY_MOVE_2R;
	static char		KEY_MOVE_2L;
	static char		KEY_MOVE_4R;
	static char		KEY_MOVE_4L;
	static char		KEY_MOVE_8R;
	static char		KEY_MOVE_8L;
	static char		KEY_RESET;
	static char		KEY_PRINT_STATUS;
	static char		RESET_ANGLE1;
	static char		RESET_ANGLE2;
	static char		RESET_ANGLE4;
	static char		RESET_ANGLE8;
	static float	MOTOR_ANGLE_OFFSET;
	static int		ANGLE_STEP;
	static int		MAX_ANGLE_1;
	static int		MIN_ANGLE_1;
	static int		MAX_ANGLE_2;
	static int		MIN_ANGLE_2;
	static int		MAX_ANGLE_4;
	static int		MIN_ANGLE_4;
	static int		MAX_ANGLE_8;
	static int		MIN_ANGLE_8;
	static int		ARM_TIMEOUT;
	static int		ICP_CLOUD_DENSITY;
	static int		ICP_MAX_ITERATIONS;

	static float	DISTANCE_TO_FOLLOW_OBJECT;

	float			Arduino::ARM_LENGTH;
	float			Arduino::MOTORS_HEIGHT;
	float			Arduino::MOTORS_WIDTH;
	float			Arduino::KINECT_HEIGHT;

	static unsigned long startTime; 

	Arduino::Arduino()
	{
		angleMotor1 = 0;
		angleMotor2 = 0;
		angleMotor4 = 0;
		angleMotor8 = 0;
		stoppedMoving = true;
		startedMoving = false;
		isMoving = false;

		icpThread = new ICPThread();
	}

	Arduino::~Arduino()
	{
		if (serial.available()){
			serial.close();
		}
	}

	bool Arduino::setup()
	{
		CHECK_ACTIVE true;

		icpThread->setup(this);

		loadXMLSettings();

		angleMotor1 = RESET_ANGLE1;
		angleMotor2 = RESET_ANGLE2;
		angleMotor4 = RESET_ANGLE4;
		angleMotor8 = RESET_ANGLE8;// La posición inicial de este motor es mirando de costado. 

		if (!serial.setup(COM_PORT, 9600)) {
			cout << "Error en setup del Serial, puerto COM: " << COM_PORT << endl;
			//return false;
		}

		EventManager::suscribe(this);

		stoppedMoving = false;
		startedMoving = false;
		
		if (IsFeatureMoveArmActive()) {
			startedMoving = true;
			// Mientras se está moviendo el brazo, nadie debería poder obtener la nube a través del método getCloud
			gTransformation->cloudMutex.lock();
			// No se debe aplicar ICP en el setup
			cloudBeforeMoving.reset();
		}

		cout << "Moviendo los motores" << endl;

		sendMotor((char) angleMotor1, ID_MOTOR_1);
		sendMotor((char) angleMotor2, ID_MOTOR_2);
		sendMotor((char) angleMotor4, ID_MOTOR_4);
		sendMotor((char) angleMotor8, ID_MOTOR_8);

		if (IsFeatureMoveArmActive()) {
			Eigen::Affine3f initTransf;
			initTransf = calculateWorldTransformation(angleMotor1,angleMotor2,angleMotor4,angleMotor8);
			gTransformation->setInitialWorldTransformation(initTransf);
			gTransformation->setWorldTransformation(initTransf);

		}

		posicion = getKinect3dCoordinates();
		mira = lookingAt();

		serial.enumerateDevices();

		return true;
	}

	void Arduino::exit() {
		CHECK_ACTIVE;
		if (serial.available()){
			serial.close();
		}

		icpThread->exit();
	}

	void Arduino::update() {
		CHECK_ACTIVE;

		if (IsFeatureMoveArmActive()) {

			if(startedMoving)
			{
				ofPoint accel = gKinect->getMksAccel();
				ofVec3f vecAccel = ofVec3f(accel.x, accel.y, accel.z);
				ofVec3f vecDiff = vecAccel - acceleration;
				float length = vecDiff.length();
				
				unsigned int elapsedTime = (unsigned int) (ofGetSystemTime() - startTime);

				if (vecDiff.length() > 0.2 || elapsedTime >= ARM_TIMEOUT)
				{
					//empezó a moverse
					startedMoving = false;
					isMoving = true;
					startTime = ofGetSystemTime();
				}
			}

			if (isMoving)
			{
				//la logica es la siguiente:
				//si el vector no se ha movido durante ARM_TIMEOUT segundos
				//se toma como que se ha dejado de mover
				unsigned int elapsedTime = (unsigned int) (ofGetSystemTime() - startTime);
				if (elapsedTime >= ARM_TIMEOUT)		// Cantidad de milisegundos para considerar que el brazo se terminó de mover
				{
					armStoppedMoving();
				}
				else
				{
					ofPoint accel = gKinect->getMksAccel();
					ofVec3f vecAccel = ofVec3f(accel.x, accel.y, accel.z);
					ofVec3f vecDiff = vecAccel - acceleration;
					float largo = vecDiff.length();
					if (vecDiff.length() > 0.4)
					{
						//se sigue moviendo
						acceleration = vecAccel;
						startTime = ofGetSystemTime();
					}
				}
			}


			if (stoppedMoving)
			{
				stoppedMoving = false;

				if (IsFeatureMoveArmActive() && !(cloudBeforeMoving.get() == NULL)) 
				{
					cloudAfterMoving = getCloudWithoutMutex(ICP_CLOUD_DENSITY);
					saveCloud("cloudAfterMoving.pcd", *cloudAfterMoving);

					icpThread->applyICP(cloudBeforeMoving,cloudAfterMoving,ICP_MAX_ITERATIONS);
				} else {
					//	Libero el mutex para que puedan invocar al método getCloud
					gTransformation->cloudMutex.unlock();

					// Además, se debe volver a dibujar en la ventana de mapping
					gTransformation->setIsWorldTransformationStable(true);
				}

			}

		}
	}

	void Arduino::draw() {
		CHECK_ACTIVE;
	}

	void Arduino::keyPressed (int key) {
		CHECK_ACTIVE;

		bool debug = true;
	
		if (debug) {
			switch (key)
			{
				case '0':
					reset(true); // Vuelve a la posición inicial, resetea la matriz de transformación y no aplica ICP
					break;
				case '6':
					moveMotor(8,80);
					break;
				case '.':
					// Recargar settings desde el archivo XML, por si se modificaron
					loadXMLSettings();
					break;
			}
			if (key == KEY_MOVE_1R) {
				moveMotor(1,angleMotor1 + ANGLE_STEP);
			} else if (key == KEY_MOVE_1L) {
				moveMotor(1,angleMotor1 - ANGLE_STEP);
			} else if (key == KEY_MOVE_2R) {
				moveMotor(2,angleMotor2 + ANGLE_STEP);
			} else if (key == KEY_MOVE_2L) {
				moveMotor(2,angleMotor2 - ANGLE_STEP);
			} else if (key == KEY_MOVE_4R) {
				moveMotor(4,angleMotor4 + ANGLE_STEP);
			} else if (key == KEY_MOVE_4L) {
				moveMotor(4,angleMotor4 - ANGLE_STEP);
			} else if (key == KEY_MOVE_8R) {
				moveMotor(8,angleMotor8 + ANGLE_STEP);
			} else if (key == KEY_MOVE_8L) {
				moveMotor(8,angleMotor8 - ANGLE_STEP);
			} 
			else if (key == KEY_RESET) {
				reset(false);
			}
			else if (key == KEY_PRINT_STATUS) {
				cout << read() << endl;
				cout << "motor 1: " << angleMotor1 << endl;
				cout << "motor 2: " << angleMotor2 << endl;
				cout << "motor 4: " << angleMotor4 << endl;
				cout << "motor 8: " << angleMotor8 << endl;
			}		
		}
	}


	void Arduino::reset(bool forceReset)
	{
		CHECK_ACTIVE;

		cout << "reset" << endl;

		loadXMLSettings();

		if (IsFeatureMoveArmActive()) {
			armStartedMoving(forceReset);	

			// No se debe aplicar ICP en el reset; sirve para "volver a una posición segura"
//			cloudBeforeMoving.reset();
		}

/*		if (IsFeatureMoveArmActive()) {
			startedMoving = true;
			if (!forceReset) {		// Si debo forzar el reset, no se debe invocar al lock, es por que ya quedó en lock el cloudMutex
				// Mientras se está moviendo el brazo, nadie debería poder obtener la nube a través del método getCloud
				gTransformation->cloudMutex.lock();
			}
			// No se debe aplicar ICP en el reset; sirve para "volver a una posición segura"
			cloudBeforeMoving.reset();
		}
*/
		if (RESET_ANGLE1 != ANGLE_UNDEFINED) {
			angleMotor1 = RESET_ANGLE1;
			sendMotor((char) angleMotor1, ID_MOTOR_1);
		}
		if (RESET_ANGLE2 != ANGLE_UNDEFINED) {
			angleMotor2 = RESET_ANGLE2;
			sendMotor((char) angleMotor2, ID_MOTOR_2);
		}
		if (RESET_ANGLE4 != ANGLE_UNDEFINED) {
			angleMotor4 = RESET_ANGLE4;
			sendMotor((char) angleMotor4, ID_MOTOR_4);
		}
		if (RESET_ANGLE8 != ANGLE_UNDEFINED) {
			angleMotor8 = RESET_ANGLE8;
			sendMotor((char) angleMotor8, ID_MOTOR_8);
		}

		if (IsFeatureMoveArmActive()) {
			gTransformation->setInitialWorldTransformation(calculateWorldTransformation(angleMotor1,angleMotor2,angleMotor4,angleMotor8));
		} else {
			gTransformation->setWorldTransformation(gTransformation->getInitialWorldTransformation());
		}

		//	Libero el mutex para que puedan invocar al método getCloud, por si quedó en lock
//		gTransformation->cloudMutex.unlock();
	}

	void Arduino::sendMotor(int value, int id)
	{
		//value += MOTOR_ANGLE_OFFSET;
		if (value < 0){
			value = -value;
			value |= 1 << 7; //MAGIC!
		}
		char id_char = (char) id;
		serial.writeByte(id_char);
		serial.writeByte(value);
		// Comienzo a contar cuanto tiempo pasó desde que se envía la señal para que se muevan los motores 
		startTime = ofGetSystemTime();
	}

	signed int* Arduino::motorAngles() const
	{
		signed int* result = new signed int[4];
		result[0] = angleMotor1;
		result[1] = angleMotor2;
		result[2] = angleMotor4;
		result[3] = angleMotor8;
		return result;
	}

	char* Arduino::read()
	{
		char* result;
		int byteCount = serial.available();
		if (byteCount > 0) {
			result = new char[byteCount];
			unsigned char byteRead = 0;
			int i = 0;
			while(serial.readBytes(&byteRead, 1) > 0) {
				result[i] = byteRead;
				i++;
			}
			result[i] = 0;
		}
		else
		{
			result = new char[1];
			result[0] = '\0';
		}
		return result;
	}

	ofVec3f Arduino::getKinect3dCoordinates()
	{
		return gTransformation->getKinectEyeCoordinates();
	}

	void Arduino::setArm3dCoordinates(float x, float y, float z, bool setArmStartedMoving)
	{
		// Setear las coordenadas de la posición donde estará el motor8 (el de más abajo del Kinect)
		//		en coordenadas de mundo
		signed int _angleMotor2 = round(atan(z/x) * 180.0f / M_PI);			//el de la base, x no deberia ser 0 nunca
		signed int _angleMotor1 = 0;
		if (y != 0) {
			if (y > 0)
			{
				_angleMotor1 = (int)round(asin(y/ARM_LENGTH) * 180.0f / M_PI);
			}
			else
			{
				_angleMotor1 = -(int)round(asin(-y/ARM_LENGTH) * 180.0f / M_PI);
			}
		} else {
			_angleMotor1 = 0;
		}
		if (!inRange(_angleMotor1, MIN_ANGLE_1, MAX_ANGLE_1))
		{
			return;
		}

		if (!inRange(_angleMotor2, MIN_ANGLE_2, MAX_ANGLE_2))
		{
			return;
		}


		if (IsFeatureMoveArmActive()) {
			if (gTransformation->getIsWorldTransformationStable()) {
				if (setArmStartedMoving)
					armStartedMoving(false);	
			} else {
				posicion = getKinect3dCoordinates();
				return;
			}
		}

		angleMotor1 =_angleMotor1;
		angleMotor2 =_angleMotor2;

		sendMotor(angleMotor1, ID_MOTOR_1);
		sendMotor(angleMotor2, ID_MOTOR_2);

		if (IsFeatureMoveArmActive()) {
			calculateWorldTransformation(angleMotor1,angleMotor2,angleMotor4,angleMotor8);
		}
		posicion = getKinect3dCoordinates(); //ofVec3f(x, y, z);
	}

	ofVec3f Arduino::setArm3dCoordinates(const ofVec3f& position, bool setArmStartedMoving)
	{
		// Setear las coordenadas de la posición donde estará el motor8 (el de más abajo del Kinect)
		// en coordenadas de mundo
		// wrapper para posicionar desde un ofVec3f
		ofVec3f bestFit = bestFitForArmSphere(position);
		setArm3dCoordinates(bestFit.x, bestFit.y, bestFit.z, setArmStartedMoving);
		return bestFit;
	}

	ofVec3f Arduino::bestFitForArmSphere(const ofVec3f& p)
	{
		//la lógica es pasar el punto que viene a un punto en coordenadas esféricas
		//reducir el r y pasarlo nuevamente a coordenadas cartesianas.
		Line3D armLine(ofVec3f(0, 0, 0), p);
		return armLine.calculateValue(Arduino::ARM_LENGTH / armLine.segmentLength());
	}

	ofVec3f	Arduino::lookAt(const ofVec3f& point)
	{
		ofVec3f miraHorizonte (ARM_LENGTH + 0.10, - KINECT_HEIGHT - MOTORS_HEIGHT, 0.0);
		ofVec3f posInicialKinect (ARM_LENGTH, - KINECT_HEIGHT - MOTORS_HEIGHT, 0.0);
		//posicion = getKinect3dCoordinates();

		//TODO: tener el cuenta la traslacion del grueso de los motores de la punta
		//posicion = donde se encuentra ubicado
		//mira = donde estoy mirando ATM
		Eigen::Vector3f axisY (0, 1, 0);
		Eigen::Affine3f rotationY;
		float angleMotor2Rad = ofDegToRad(angleMotor2); // Motor de abajo del brazo, con la varilla "vertical"
		rotationY = Eigen::AngleAxis<float>(angleMotor2Rad, axisY);				// Debe ser positivo

		Eigen::Vector3f axisZ (0, 0, 1);
		Eigen::Affine3f rotationZ;
		float angleMotor1Rad = ofDegToRad(angleMotor1);	// Motor que mueve la varilla "horizontal"
		rotationZ = Eigen::AngleAxis<float>(-angleMotor1Rad, axisZ);			// Debe ser negativo

		Eigen::Affine3f composedMatrix;
		composedMatrix = rotationY * rotationZ;
		
		Eigen::Vector3f ePoint = Eigen::Vector3f(point.x, point.y, point.z);
		Eigen::Vector3f eMira = Eigen::Vector3f(miraHorizonte.x, miraHorizonte.y, miraHorizonte.z);
		Eigen::Vector3f ePosicion = Eigen::Vector3f(posInicialKinect.x, posInicialKinect.y, posInicialKinect.z);

		Eigen::Vector3f etPoint = composedMatrix * ePoint;
		Eigen::Vector3f etMira = /*composedMatrix * */ eMira;
		Eigen::Vector3f etPosicion = /*composedMatrix * */ ePosicion;

		ofVec3f tPoint = ofVec3f(etPoint.x(), etPoint.y(), etPoint.z());
		ofVec3f tMira = ofVec3f(etMira.x(), etMira.y(), etMira.z());
		ofVec3f tPosicion = ofVec3f(etPosicion.x(), etPosicion.y(), etPosicion.z());

		ofVec3f origen = ofVec3f(0, 0, 0);
		ofVec3f ejeY = ofVec3f(0, 1, 0);
		Plane3D horizontal = Plane3D(origen, ejeY); //lo defino con el plano que xz y normal y

		ofVec3f phtPoint = horizontal.project(tPoint);
		ofVec3f phtMira = horizontal.project(tMira);
		ofVec3f phtPosicion = horizontal.project(tPosicion);

		ofVec3f hvMira = phtMira - phtPosicion;
		ofVec3f hvPoint = phtPoint - phtPosicion;

		float anguloH = 0;

		ofVec3f hNormal = horizontal.getNormal();
		if (!(abs(hvMira.length()) <= MATH_EPSILON || abs(hvPoint.length()) <= MATH_EPSILON	
				|| (abs(hvMira.dot(hNormal) - hvMira.length()*hNormal.length()) <= MATH_EPSILON)
				|| (abs(hvPoint.dot(hNormal) - hvPoint.length()*hNormal.length()) <= MATH_EPSILON) ))
		{
				//if (length de alguno de los vectores < MATH_EPSILON || alguno de los vectores es "casi" paralelo a la normal del plano)
				anguloH = hvMira.angle(hvPoint);//motor 8
				if (point.z < 0)
				{
					anguloH *= -1;
				}
		}		 

		ofVec3f ejeZ = ofVec3f(0, 0, 1);
		ejeZ = ejeZ.rotate(-anguloH, ejeY);
		Plane3D vertical = Plane3D(tPosicion, ejeZ);

		ofVec3f miraTrans = ofVec3f(tMira.x - posInicialKinect.x, tMira.y - posInicialKinect.y, tMira.z - posInicialKinect.z); 
		miraTrans = miraTrans.rotate(-anguloH, ejeY);
		tMira = ofVec3f(miraTrans.x + posInicialKinect.x, miraTrans.y + posInicialKinect.y, miraTrans.z + posInicialKinect.z);

		ofVec3f pvtPoint = vertical.project(tPoint);
		ofVec3f pvtMira = vertical.project(tMira);
		ofVec3f pvtPosicion = vertical.project(tPosicion);

		ofVec3f vvMira = pvtMira - pvtPosicion;
		ofVec3f vvPoint = pvtPoint - pvtPosicion;

		float anguloV = 0;
		ofVec3f vNormal = vertical.getNormal();
		if (!(abs(vvMira.length()) <= MATH_EPSILON || abs(vvPoint.length()) <= MATH_EPSILON	
				|| (abs(vvMira.dot(vNormal) - vvMira.length()*vNormal.length()) <= MATH_EPSILON)
				|| (abs(vvPoint.dot(vNormal) - vvPoint.length()*vNormal.length()) <= MATH_EPSILON) ))
		{
				//if (length de alguno de los vectores < MATH_EPSILON || alguno de los vectores es "casi" paralelo a la normal del plano)	
				anguloV = vvMira.angle(vvPoint);//motor 4
				if (point.y > (- KINECT_HEIGHT - MOTORS_HEIGHT) )
				{
					anguloV *= -1;
				}
		}		 	

		if (!inRange(int(anguloV), MIN_ANGLE_4, MAX_ANGLE_4))
		{
			return NULL;
		}
		if (!inRange(int(anguloH), MIN_ANGLE_8, MAX_ANGLE_8))
		{
			return NULL;
		}

		if (IsFeatureMoveArmActive())
		{
			if (gTransformation->getIsWorldTransformationStable())
			{
				armStartedMoving(false);
			}
			else
			{
				return posicion = getKinect3dCoordinates();
			}
		}

		angleMotor4 = anguloV;
		angleMotor8 = anguloH;

		//mira_actual = point;
		mira = point;

		sendMotor(angleMotor4, ID_MOTOR_4);
		sendMotor(angleMotor8, ID_MOTOR_8);

		if (IsFeatureMoveArmActive()) {
			calculateWorldTransformation(angleMotor1,angleMotor2,angleMotor4,angleMotor8);
		}

		return lookingAt();
	}

	bool Arduino::isActive()
	{
		return IsFeatureArduinoActive();
	}

	ofVec3f	Arduino::lookingAt()
	{
		return gTransformation->getKinectDirectionCoordinates();
	}

	Eigen::Affine3f Arduino::calculateWorldTransformation(float angle1, float angle2, float angle4, float angle8)
	{
		float angleMotor1Rad = ofDegToRad(angle1);		// Motor que mueve la varilla "horizontal"
		float angleMotor2Rad = ofDegToRad(angle2);		// Motor de abajo del brazo, con la varilla "vertical"
		float angleMotor4Rad = ofDegToRad(angle4);		// Motor de los de la punta, el de más arriba, sobre el que está enganchado la base del Kinect
		float angleMotor8Rad = ofDegToRad(angle8 - 90);	// Motor de los de la punta, el de más abajo. La posición inicial de referencia será 90 grados.
//		float angleTiltKinectRad = ofDegToRad(TILT_ANGLE);

		//todas las matrices segun: http://pages.cs.brandeis.edu/~cs155/Lecture_07_6.pdf

		Eigen::Vector3f axisX (1, 0, 0);
		Eigen::Vector3f axisY (0, 1, 0);
		Eigen::Vector3f axisZ (0, 0, 1);

//		Eigen::Affine3f tAlturaBrazo;
//		tAlturaBrazo = Eigen::Translation<float, 3>(0, -ARM_HEIGHT, 0);

		Eigen::Affine3f rMotor2;
		rMotor2 = Eigen::AngleAxis<float>(-angleMotor2Rad, axisY);
		
		Eigen::Affine3f rMotor1;
		rMotor1 = Eigen::AngleAxis<float>(angleMotor1Rad, axisZ);

		Eigen::Affine3f tLargoBrazo;
		tLargoBrazo = Eigen::Translation<float, 3>(ARM_LENGTH, 0, 0);//capaz se puede combinar con el anterior, no?

		Eigen::Affine3f rMotor8;
		rMotor8 = Eigen::AngleAxis<float>(-angleMotor8Rad, axisY);

		Eigen::Affine3f tMotores48;
		tMotores48 = Eigen::Translation<float, 3>(0, -MOTORS_HEIGHT, MOTORS_WIDTH);
			
		Eigen::Affine3f rMotor4;
		rMotor4 = Eigen::AngleAxis<float>(angleMotor4Rad, axisX);

/*		Eigen::Affine3f tAlturaMotor4AlKinect;
		tAlturaMotor4AlKinect = Eigen::Translation<float, 3>(0, -KINECT_MOTOR_HEIGHT, 0);

		Eigen::Affine3f rTiltKinect;
		rTiltKinect = Eigen::AngleAxis<float>(angleTiltKinectRad, axisX);
*/
		Eigen::Affine3f tAlturaAlKinect;
		tAlturaAlKinect = Eigen::Translation<float, 3>(0.012, -KINECT_HEIGHT, 0);

		//con estas tres matrices tengo todas las rotaciones que preciso, ahora
		//preciso hallar la traslacion de altura donde esta la camara
		//y luego la traslacion a lo largo del brazo
		
		Eigen::Affine3f composedMatrix;
//		composedMatrix = tAlturaBrazo * (rMotor2 * (rMotor1 *  (tLargoBrazo * (rMotor8 * (tMotores48 * (rMotor4 * (tAlturaMotor4AlKinect * (rTiltKinect * tAlturaKinect))))))) /*)*/;
		composedMatrix = rMotor2 * (rMotor1 *  (tLargoBrazo * (rMotor8 * (tMotores48 * (rMotor4 * tAlturaAlKinect))))) ;

		gTransformation->setWorldTransformation(composedMatrix);

		/*
		ofVec3f eye = transformPoint(ofVec3f(0,0,0),composedMatrix);
		cout << "eye transformado = " << ofVecToString(eye) << endl;

		ofVec3f mesa = transformPoint(ofVec3f(0,0.5,1.0),composedMatrix);
		cout << "mesa transformado = " << ofVecToString(mesa) << endl;

		ofVec3f cent(0.16555759, 0.10312909, 1.2223474);
		ofVec3f centroide = transformPoint(cent,composedMatrix);
		cout << "centroide transformado = " << ofVecToString(centroide) << endl;
		*/
		return composedMatrix;

	}

	void Arduino::objectUpdated(const IObjectPtr& object)
	{
		if (object->getId() == idObjectToFollow)
		{
			if (centerOfFollowingObject.distance(object->getCenter()) > DISTANCE_TO_FOLLOW_OBJECT)
			{
				lookAt(object->getCenter());
			}
		}
	}

	
	void Arduino::followObject(const IObjectPtr& object)
	{
		idObjectToFollow = object->getId();
		centerOfFollowingObject = object->getCenter();
		lookAt(object->getCenter());
	}

	void Arduino::armStartedMoving(bool moveForced)
	{		
		if (IsFeatureMoveArmActive())
		{
			startedMoving = true;
			ofPoint accel = gKinect->getMksAccel();
			acceleration = ofVec3f(accel.x, accel.y, accel.z);
			gTransformation->setIsWorldTransformationStable(false);

			if (gModel->getTable() != NULL)
			{ 
				// Obtener nube antes de mover los motores
				if (moveForced)
					cloudBeforeMoving = getCloudWithoutMutex(ICP_CLOUD_DENSITY);
				else
					cloudBeforeMoving = getCloud(ICP_CLOUD_DENSITY);
				saveCloud("cloudBeforeMoving.pcd", *cloudBeforeMoving);
			}
			else
			{
				// No aplicar ICP si no se tiene mesa detectada
				cloudBeforeMoving.reset();
			}

			if (!moveForced) // Si debo forzar el reset, no se debe invocar al lock, es por que ya quedó en lock el cloudMutex
			{
				// Mientras se está moviendo el brazo, nadie debería poder obtener la nube a través del método getCloud
				gTransformation->cloudMutex.lock();
			}
		}

	}

	void Arduino::armStoppedMoving()
	{
		if (IsFeatureMoveArmActive()) {
			unsigned int elapsedTime = (unsigned int) (ofGetSystemTime() - startTime);
			cout << "Brazo se dejo de mover luego de " << elapsedTime << "ms" << endl ;	

			startedMoving = false;
			isMoving = false;
			stoppedMoving = true;
		}
	}

	void Arduino::stopFollowing()
	{
		idObjectToFollow = -2; //-2 nunca va a ser un ID porque son mayores que 0, -1 es la mesa
	}

	void Arduino::loadXMLSettings()
	{
		ofxXmlSettings XML;
		if(XML.loadFile("Arduino_Config.xml")) {

			COM_PORT = XML.getValue(ARDUINO_CONFIG "COM_PORT", "COM3");
			ANGLE_STEP = XML.getValue(ARDUINO_CONFIG "ANGLE_STEP", 10);

			RESET_ANGLE1 = XML.getValue(ARDUINO_CONFIG "RESET_ANGLE1", ANGLE_DEFAULT);
			RESET_ANGLE2 = XML.getValue(ARDUINO_CONFIG "RESET_ANGLE2", ANGLE_DEFAULT);
			RESET_ANGLE4 = XML.getValue(ARDUINO_CONFIG "RESET_ANGLE4", ANGLE_DEFAULT);
			RESET_ANGLE8 = XML.getValue(ARDUINO_CONFIG "RESET_ANGLE8", ANGLE_DEFAULT);

			KEY_RESET = XML.getValue(ARDUINO_CONFIG "KEY_RESET", KEY_UNDEFINED).c_str()[0];
			KEY_PRINT_STATUS = XML.getValue(ARDUINO_CONFIG "KEY_PRINT_STATUS", KEY_UNDEFINED).c_str()[0];

			KEY_MOVE_1R = XML.getValue(ARDUINO_CONFIG "KEY_MOVE_1R", KEY_UNDEFINED).c_str()[0];
			KEY_MOVE_1L = XML.getValue(ARDUINO_CONFIG "KEY_MOVE_1L", KEY_UNDEFINED).c_str()[0];
			KEY_MOVE_2R = XML.getValue(ARDUINO_CONFIG "KEY_MOVE_2R", KEY_UNDEFINED).c_str()[0];
			KEY_MOVE_2L = XML.getValue(ARDUINO_CONFIG "KEY_MOVE_2L", KEY_UNDEFINED).c_str()[0];
			KEY_MOVE_4R = XML.getValue(ARDUINO_CONFIG "KEY_MOVE_4R", KEY_UNDEFINED).c_str()[0];
			KEY_MOVE_4L = XML.getValue(ARDUINO_CONFIG "KEY_MOVE_4L", KEY_UNDEFINED).c_str()[0];
			KEY_MOVE_8R = XML.getValue(ARDUINO_CONFIG "KEY_MOVE_8R", KEY_UNDEFINED).c_str()[0];
			KEY_MOVE_8L = XML.getValue(ARDUINO_CONFIG "KEY_MOVE_8L", KEY_UNDEFINED).c_str()[0];

			MOTOR_ANGLE_OFFSET	= XML.getValue(ARDUINO_CONFIG "MOTOR_ANGLE_OFFSET", MOTOR_ANGLE_OFFSET_DEFAULT);

			ARM_LENGTH = XML.getValue(ARDUINO_CONFIG "ARM_LENGTH", 0.354);			
			MOTORS_HEIGHT = XML.getValue(ARDUINO_CONFIG "MOTORS_HEIGHT", 0.056);
			MOTORS_WIDTH = XML.getValue(ARDUINO_CONFIG "MOTORS_WIDTH", 0.015);
			KINECT_HEIGHT = XML.getValue(ARDUINO_CONFIG "KINECT_HEIGHT", 0.116);

			MAX_ANGLE_1 = XML.getValue(ARDUINO_CONFIG "MAX_ANGLE_1", 0);
			MIN_ANGLE_1 = XML.getValue(ARDUINO_CONFIG "MIN_ANGLE_1", 0);
			MAX_ANGLE_2 = XML.getValue(ARDUINO_CONFIG "MAX_ANGLE_2", 0);
			MIN_ANGLE_2 = XML.getValue(ARDUINO_CONFIG "MIN_ANGLE_2", 0);
			MAX_ANGLE_4 = XML.getValue(ARDUINO_CONFIG "MAX_ANGLE_4", 0);
			MIN_ANGLE_4 = XML.getValue(ARDUINO_CONFIG "MIN_ANGLE_4", 0);
			MAX_ANGLE_8 = XML.getValue(ARDUINO_CONFIG "MAX_ANGLE_8", 0);
			MIN_ANGLE_8 = XML.getValue(ARDUINO_CONFIG "MIN_ANGLE_8", 0);

			ARM_TIMEOUT = XML.getValue(ARDUINO_CONFIG "ARM_TIMEOUT", 2000);
			ICP_CLOUD_DENSITY = XML.getValue(ARDUINO_CONFIG "ICP_CLOUD_DENSITY", Constants::CLOUD_STRIDE());
			ICP_MAX_ITERATIONS = XML.getValue(ARDUINO_CONFIG "ICP_MAX_ITERATIONS", 20);

			DISTANCE_TO_FOLLOW_OBJECT = XML.getValue(ARDUINO_CONFIG "DISTANCE_TO_FOLLOW_OBJECT", 0.25);
		}
	}

	ofVec3f Arduino::moveMotor(int motorId, signed int degrees)
	{
		ofVec3f error = ofVec3f(INT_MAX, INT_MAX, INT_MAX);
		signed int angle1 = angleMotor1, angle2 = angleMotor2, angle4 = angleMotor4, angle8 = angleMotor8;

		if (motorId == ID_MOTOR_1)
		{
			if (!inRange(degrees, MIN_ANGLE_1, MAX_ANGLE_1))
			{
				return error;
			}
			else
			{
				angle1 = degrees;
			}
		}
		if (motorId == ID_MOTOR_2)
		{
			if (!inRange(degrees, MIN_ANGLE_2, MAX_ANGLE_2))
			{
				return error;
			}
			else
			{
				angle2 = degrees;
			}
		}
		if (motorId == ID_MOTOR_4)
		{
			if (!inRange(degrees, MIN_ANGLE_4, MAX_ANGLE_4))
			{
				return error;
			}
			else
			{
				angle4 = degrees;
			}
		}
		if (motorId == ID_MOTOR_8)
		{
			if (!inRange(degrees, MIN_ANGLE_8, MAX_ANGLE_8))
			{
				return error;
			}
			else
			{
				angle8 = degrees;
			}
		}
		
		if (IsFeatureMoveArmActive())
		{
			if (gTransformation->getIsWorldTransformationStable())
			{
				angleMotor1 = angle1;
				angleMotor2 = angle2;
				angleMotor4 = angle4;
				angleMotor8 = angle8;
				armStartedMoving(false);
			}
			else
			{
				return posicion = getKinect3dCoordinates();
			}
		}

		sendMotor(angleMotor1, ID_MOTOR_1);
		sendMotor(angleMotor2, ID_MOTOR_2);
		sendMotor(angleMotor4, ID_MOTOR_4);
		sendMotor(angleMotor8, ID_MOTOR_8);

		if (IsFeatureMoveArmActive())
		{
			calculateWorldTransformation(angleMotor1,angleMotor2,angleMotor4,angleMotor8);
		}
		posicion = getKinect3dCoordinates(); //ofVec3f(x, y, z);

		return posicion;
	}

}
