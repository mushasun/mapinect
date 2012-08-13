#include "Arduino.h"

#include "Feature.h"
#include "ofxXmlSettings.h"
#include <direct.h> // for getcwd
#include "Plane3D.h"
#include "utils.h"
#include "Globals.h"

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
	static int		ELAPSED_TIME_ARM_STOPPED_MOVING;
	static int		ICP_CLOUD_DENSITY;
	static int		ICP_MAX_ITERATIONS;

	float			Arduino::ARM_LENGTH;
	float			Arduino::KINECT_HEIGHT;
	float			Arduino::MOTORS_HEIGHT;

	static unsigned long startTime; 

	Arduino::Arduino()
	{
		angleMotor1 = 0;
		angleMotor2 = 0;
		angleMotor4 = 0;
		angleMotor8 = 0;
		stoppedMoving = true;
	}

	Arduino::~Arduino()
	{
		if (serial.available()){
			serial.close();
		}
	}

	bool Arduino::setup()
	{
		CHECK_ACTIVE false;

		icpThread.setup();

		loadXMLSettings();

		moveMotor(ID_MOTOR_1, RESET_ANGLE1);
		moveMotor(ID_MOTOR_2, RESET_ANGLE2);
		moveMotor(ID_MOTOR_4, RESET_ANGLE4);
		moveMotor(ID_MOTOR_8, RESET_ANGLE8);// La posición inicial de este motor es mirando de costado. 

		if (!serial.setup(COM_PORT, 9600)) {
			cout << "Error en setup del Serial, puerto COM: " << COM_PORT << endl;
			//return false;
		}

		EventManager::suscribe(this);

		stoppedMoving = false;
		armMoving = false;
		
		if (IsFeatureMoveArmActive()) {
			armMoving = true;
			// Mientras se está moviendo el brazo, nadie debería poder obtener la nube a través del método getCloud
			gTransformation->cloudMutex.lock();
			// No se debe aplicar ICP en el setup
			cloudBeforeMoving.reset();
		}

		sendMotor(angleMotor1, ID_MOTOR_1);
		sendMotor(angleMotor2, ID_MOTOR_2);
		sendMotor(angleMotor4, ID_MOTOR_4);
		sendMotor(angleMotor8, ID_MOTOR_8);

		if (IsFeatureMoveArmActive()) {
			gTransformation->setInitialWorldTransformation(calculateWorldTransformation(angleMotor1,angleMotor2,angleMotor4,angleMotor8));
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

		icpThread.exit();
	}

	void Arduino::update() {
		CHECK_ACTIVE;

		if (IsFeatureMoveArmActive()) {

			if (armMoving)
			{
				//la logica es la siguiente:
				//si el vector no se ha movido durante ELAPSED_TIME_ARM_STOPPED_MOVING segundos
				//se toma como que se ha dejado de mover
				unsigned int elapsedTime = (unsigned int) (ofGetSystemTime() - startTime);
				if (elapsedTime >= ELAPSED_TIME_ARM_STOPPED_MOVING)		// Cantidad de milisegundos para considerar que el brazo se terminó de mover
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

					icpThread.applyICP(cloudBeforeMoving,cloudAfterMoving,ICP_MAX_ITERATIONS);
				} else {
					//	Libero el mutex para que puedan invocar al método getCloud
					gTransformation->cloudMutex.unlock();
				}

			}

		}
	}

	void Arduino::draw() {
		CHECK_ACTIVE;
	}

	void Arduino::keyPressed (int key) {
		CHECK_ACTIVE;

		ofVec3f centroidePrueba(0.3188, 0.2454, 1.0842);

		float sin8 = 0.12533;
		float sin15 = 0.2588;
		float sin30 = 0.5;
		float sin45 = 0.7071;
		float cos8 = 0.9921;
		float cos15 = 0.9659;
		float cos30 = 0.866;
		float cos45 = 0.7071;
		switch (key)
		{
			case '9':
				lookAt(centroidePrueba);
				break;
			case '0':
				reset(); // Vuelve a la posición inicial, resetea la matriz de transformación y no aplica ICP
				//setArm3dCoordinates(ofVec3f(Arduino::ARM_LENGTH, 0, 0)); 
				break;
			case '1':
				//AngleMotor1 = -15
				setArm3dCoordinates(ofVec3f(Arduino::ARM_LENGTH*cos15, -Arduino::ARM_LENGTH*sin15, 0)); 
				break;
			case '2':
				//AngleMotor2 = 15
				setArm3dCoordinates(ofVec3f(Arduino::ARM_LENGTH*cos15, 0, Arduino::ARM_LENGTH*sin15)); 
				break;
			case '3':
				//AngleMotor8 > 90
				setArm3dCoordinates(ofVec3f(Arduino::ARM_LENGTH, 0, 0));
				break;
			case '4':
				//AngleMotor4 < 0 
				setArm3dCoordinates(ofVec3f(Arduino::ARM_LENGTH, 0, 0));
				break;
			case '5':
				//AngleMotor1 = -8
				setArm3dCoordinates(ofVec3f(Arduino::ARM_LENGTH*cos8, -Arduino::ARM_LENGTH*sin8, 0)); 
				break;
			case '.':
				// Recargar settings desde el archivo XML, por si se modificaron
				loadXMLSettings();
				break;
			case '=':
				applyICPLoadedClouds();
				break;
		}
		if (key == KEY_MOVE_1R) {
			angleMotor1 += ANGLE_STEP;
			sendMotor((char) angleMotor1, ID_MOTOR_1);
		}
		else if (key == KEY_MOVE_1L) {
			angleMotor1 -= ANGLE_STEP;
			sendMotor((char) angleMotor1, ID_MOTOR_1);
		}
		else if (key == KEY_MOVE_2R) {
			angleMotor2 += ANGLE_STEP;
			sendMotor((char) angleMotor2, ID_MOTOR_2);
		}
		else if (key == KEY_MOVE_2L) {
			angleMotor2 -= ANGLE_STEP;
			sendMotor((char) angleMotor2, ID_MOTOR_2);
		}
		else if (key == KEY_MOVE_4R) {
			angleMotor4 += ANGLE_STEP;
			sendMotor((char) angleMotor4, ID_MOTOR_4);
		}
		else if (key == KEY_MOVE_4L) {
			angleMotor4 -= ANGLE_STEP;
			sendMotor((char) angleMotor4, ID_MOTOR_4);
		}
		else if (key == KEY_MOVE_8R) {
			angleMotor8 += ANGLE_STEP;
			sendMotor((char) angleMotor8, ID_MOTOR_8);
		}
		else if (key == KEY_MOVE_8L) {
			angleMotor8 -= ANGLE_STEP;
			sendMotor((char) angleMotor8, ID_MOTOR_8);
		}
		else if (key == KEY_RESET) {
			reset();
		}
		else if (key == KEY_PRINT_STATUS) {
			cout << read() << endl;
			cout << "motor 1: " << angleMotor1 << endl;
			cout << "motor 2: " << angleMotor2 << endl;
			cout << "motor 4: " << angleMotor4 << endl;
			cout << "motor 8: " << angleMotor8 << endl;
		}
		else if (key == 'z')
		{
			setArm3dCoordinates(ofVec3f(ARM_LENGTH, -0.10, 0.10));
		}
		else if (key == 'a')
		{
			setArm3dCoordinates(ofVec3f(ARM_LENGTH, 0, 0));
		}
		else if (key == 'x')
		{
			lookAt(ofVec3f(0.35, -0.16, 0.15));
		}
		else if (key == 'c')
		{
			lookAt(ofVec3f(0.35, -0.13, 0.10));
		}
		else if (key == 's')
		{
			lookAt(ofVec3f(0.33, -KINECT_HEIGHT-MOTORS_HEIGHT, 0.1));
		}		

	}


	void Arduino::reset()
	{
		CHECK_ACTIVE;

		loadXMLSettings();

		if (IsFeatureMoveArmActive()) {
			armMoving = true;
			// Mientras se está moviendo el brazo, nadie debería poder obtener la nube a través del método getCloud
			gTransformation->cloudMutex.lock();
			// No se debe aplicar ICP en el reset; sirve para "volver a una posición segura"
			cloudBeforeMoving.reset();
		}

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
		gTransformation->cloudMutex.unlock();
	}

	const char *my_byte_to_binary(int x)
	{
		static char b[9];
		b[0] = '\0';

		int z;
		for (z = 256; z > 0; z >>= 1)
		{
			strcat(b, ((x & z) == z) ? "1" : "0");
		}

		return b;
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
		int cantidad_bytes = serial.available();
		if (cantidad_bytes > 0) {
			result = new char[cantidad_bytes];
			unsigned char lectura = 0;
			int i = 0;
			while(serial.readBytes(&lectura, 1) > 0) {
				result[i] = lectura;
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
		Eigen::Vector3f kinectPos (0.0, 0.0, 0.0);		// Posicion inicial, en Sist. de Coord Local del Kinect
		kinectPos = gTransformation->getWorldTransformation() * kinectPos;
		return ofVec3f(kinectPos.x(), kinectPos.y(), kinectPos.z());		
	}

	void Arduino::setArm3dCoordinates(float x, float y, float z)
	{
		// Setear las coordenadas de la posición donde estará el motor8 (el de más abajo del Kinect)
		//		en coordenadas de mundo
		signed int _angleMotor2 = round(atan(z/x) * 180.0f / M_PI);			//el de la base, x no deberia ser 0 nunca
		signed int _angleMotor1 = 0;
		if (y != 0) {
			if (y > 0)
			{
				_angleMotor1 = (int)round(asin(y/ARM_LENGTH) * 180.0f / M_PI); // estaba mal, era el asin
			}
			else
			{
				_angleMotor1 = -(int)round(asin(-y/ARM_LENGTH) * 180.0f / M_PI); // estaba mal, era el asin
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

		angleMotor1 =_angleMotor1;
		angleMotor2 =_angleMotor2;

		if (IsFeatureMoveArmActive()) {
			armStartedMoving();
		}

		sendMotor(angleMotor1, ID_MOTOR_1);
		sendMotor(angleMotor2, ID_MOTOR_2);

		if (IsFeatureMoveArmActive()) {
			calculateWorldTransformation(angleMotor1,angleMotor2,angleMotor4,angleMotor8);
		}
		posicion = getKinect3dCoordinates(); //ofVec3f(x, y, z);
	}

	ofVec3f Arduino::setArm3dCoordinates(const ofVec3f& position)
	{
		// Setear las coordenadas de la posición donde estará el motor8 (el de más abajo del Kinect)
		// en coordenadas de mundo
		// wrapper para posicionar desde un ofVec3f
		ofVec3f bestFit = bestFitForArmSphere(position);
		setArm3dCoordinates(bestFit.x, bestFit.y, bestFit.z);
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
		
		Eigen::Vector3f e_point = Eigen::Vector3f(point.x, point.y, point.z);
		Eigen::Vector3f e_mira = Eigen::Vector3f(miraHorizonte.x, miraHorizonte.y, miraHorizonte.z);
		Eigen::Vector3f e_posicion = Eigen::Vector3f(posInicialKinect.x, posInicialKinect.y, posInicialKinect.z);

		Eigen::Vector3f et_point = composedMatrix * e_point;
		Eigen::Vector3f et_mira = /*composedMatrix * */ e_mira;
		Eigen::Vector3f et_posicion = /*composedMatrix * */ e_posicion;

		ofVec3f t_point = ofVec3f(et_point.x(), et_point.y(), et_point.z());
		ofVec3f t_mira = ofVec3f(et_mira.x(), et_mira.y(), et_mira.z());
		ofVec3f t_posicion = ofVec3f(et_posicion.x(), et_posicion.y(), et_posicion.z());

		ofVec3f origen = ofVec3f(0, 0, 0);
		ofVec3f eje_y = ofVec3f(0, 1, 0);
		Plane3D horizontal = Plane3D(origen, eje_y); //lo defino con el plano que xz y normal y

		ofVec3f pht_point = horizontal.project(t_point);
		ofVec3f pht_mira = horizontal.project(t_mira);
		ofVec3f pht_posicion = horizontal.project(t_posicion);

		ofVec3f hv_mira = pht_mira - pht_posicion;
		ofVec3f hv_point = pht_point - pht_posicion;

		float angulo_h = 0;

		ofVec3f h_normal = horizontal.getNormal();
		if (!(abs(hv_mira.length()) <= MATH_EPSILON || abs(hv_point.length()) <= MATH_EPSILON	
				|| (abs(hv_mira.dot(h_normal) - hv_mira.length()*h_normal.length()) <= MATH_EPSILON)
				|| (abs(hv_point.dot(h_normal) - hv_point.length()*h_normal.length()) <= MATH_EPSILON) )) {
				//if (length de alguno de los vectores < MATH_EPSILON || alguno de los vectores es "casi" paralelo a la normal del plano)
				angulo_h = hv_mira.angle(hv_point);//motor 8
				if (point.z < 0)
				{
					angulo_h *= -1;
				}
		}		 

		ofVec3f eje_z = ofVec3f(0, 0, 1);
		eje_z = eje_z.rotate(-angulo_h, eje_y);
		Plane3D vertical = Plane3D(t_posicion, eje_z);

		ofVec3f mira_trans = ofVec3f(t_mira.x - posInicialKinect.x, t_mira.y - posInicialKinect.y, t_mira.z - posInicialKinect.z); 
		mira_trans = mira_trans.rotate(-angulo_h, eje_y);
		t_mira = ofVec3f(mira_trans.x + posInicialKinect.x, mira_trans.y + posInicialKinect.y, mira_trans.z + posInicialKinect.z);

		ofVec3f pvt_point = vertical.project(t_point);
		ofVec3f pvt_mira = vertical.project(t_mira);
		ofVec3f pvt_posicion = vertical.project(t_posicion);

		ofVec3f vv_mira = pvt_mira - pvt_posicion;
		ofVec3f vv_point = pvt_point - pvt_posicion;

		float angulo_v = 0;
		ofVec3f v_normal = vertical.getNormal();
		if (!(abs(vv_mira.length()) <= MATH_EPSILON || abs(vv_point.length()) <= MATH_EPSILON	
				|| (abs(vv_mira.dot(v_normal) - vv_mira.length()*v_normal.length()) <= MATH_EPSILON)
				|| (abs(vv_point.dot(v_normal) - vv_point.length()*v_normal.length()) <= MATH_EPSILON) )) {
				//if (length de alguno de los vectores < MATH_EPSILON || alguno de los vectores es "casi" paralelo a la normal del plano)	
				angulo_v = vv_mira.angle(vv_point);//motor 4
				if (point.y > (- KINECT_HEIGHT - MOTORS_HEIGHT) )
				{
					angulo_v *= -1;
				}
		}		 	

		if (!inRange(int(angulo_v), MIN_ANGLE_4, MAX_ANGLE_4))
		{
			return NULL;
		}
		if (!inRange(int(angulo_h), MIN_ANGLE_8, MAX_ANGLE_8))
		{
			return NULL;
		}

		angleMotor4 = angulo_v;
		angleMotor8 = angulo_h;

		//mira_actual = point;
		mira = point;

		if (IsFeatureMoveArmActive()) {
			armStartedMoving();
		}

		sendMotor(angleMotor4, ID_MOTOR_4);
		sendMotor(angleMotor8, ID_MOTOR_8);

		if (IsFeatureMoveArmActive()) {
			calculateWorldTransformation(angleMotor1,angleMotor2,angleMotor4,angleMotor8);
		}

		return NULL;
	}

	bool Arduino::isActive()
	{
		return IsFeatureArduinoActive();
	}

	ofVec3f	Arduino::lookingAt()
	{
		Eigen::Vector3f kinectMira (0.0, 0.0, 0.10);		// Mira inicial, en Sist. de Coord Local del Kinect
		kinectMira = gTransformation->getWorldTransformation() * kinectMira;
		return ofVec3f(kinectMira.x(), kinectMira.y(), kinectMira.z());	
	}

	Eigen::Affine3f Arduino::calculateWorldTransformation(float angle1, float angle2, float angle4, float angle8)
	{
		float angleMotor1Rad = ofDegToRad(angle1);		// Motor que mueve la varilla "horizontal"
		float angleMotor2Rad = ofDegToRad(angle2);		// Motor de abajo del brazo, con la varilla "vertical"
		float angleMotor4Rad = ofDegToRad(angle4);		// Motor de los de la punta, el de más arriba, sobre el que está enganchado la base del Kinect
		float angleMotor8Rad = ofDegToRad(angle8 - 90);	// Motor de los de la punta, el de más abajo. La posición inicial de referencia será 90 grados.

		//todas las matrices segun: http://pages.cs.brandeis.edu/~cs155/Lecture_07_6.pdf

		Eigen::Vector3f axisX (1, 0, 0);
		Eigen::Vector3f axisY (0, 1, 0);
		Eigen::Vector3f axisZ (0, 0, 1);

		Eigen::Affine3f rotationY;
		rotationY = Eigen::AngleAxis<float>(-angleMotor2Rad, axisY);
		
		Eigen::Affine3f rotationZ;
		rotationZ = Eigen::AngleAxis<float>(angleMotor1Rad, axisZ);

		Eigen::Affine3f translationX;
		translationX = Eigen::Translation<float, 3>(ARM_LENGTH, 0, 0);//capaz se puede combinar con el anterior, no?

		Eigen::Affine3f rotationY2;
		rotationY2 = Eigen::AngleAxis<float>(-angleMotor8Rad, axisY);

		Eigen::Affine3f translationY;
		translationY = Eigen::Translation<float, 3>(0, -MOTORS_HEIGHT, 0);

		Eigen::Affine3f rotationX;
		rotationX = Eigen::AngleAxis<float>(angleMotor4Rad, axisX);

		Eigen::Affine3f translationY2;
		translationY2 = Eigen::Translation<float, 3>(0, -KINECT_HEIGHT, 0);

		//con estas tres matrices tengo todas las rotaciones que preciso, ahora
		//preciso hallar la traslacion de altura donde esta la camara
		//y luego la traslacion a lo largo del brazo
		
		Eigen::Affine3f composedMatrix;
		composedMatrix = rotationY * (rotationZ *  (translationX * (rotationY2 * (translationY * (rotationX * translationY2)))));

		gTransformation->setWorldTransformation(composedMatrix);

		return composedMatrix;

	}

	void Arduino::objectUpdated(const IObjectPtr& object)
	{
		if (object->getId() == idObjectToFollow)
		{
			if (centerOfFollowingObject.distance(object->getCenter()) > 0.05)
			{
				lookAt(object->getCenter());
			}
		}
	}

	
	void Arduino::followObject(const IObjectPtr& object)
	{
		idObjectToFollow = object->getId();
		centerOfFollowingObject = object->getCenter();
		//lookAt(object->getCenter());
	}

	void Arduino::applyICPLoadedClouds() 
	{
		PCPtr cloudBefore = loadCloud("cloudBeforeMoving2.pcd");
		PCPtr cloudAfter = loadCloud("cloudAfterMoving2.pcd");

		icpThread.applyICP(cloudBeforeMoving,cloudAfterMoving,ICP_MAX_ITERATIONS);
	}

	void Arduino::armStartedMoving()
	{		
		if (IsFeatureMoveArmActive()) {
			armMoving = true;
			gTransformation->setIsWorldTransformationStable(false);

			// Obtener nube antes de mover los motores
			cloudBeforeMoving = getCloud(ICP_CLOUD_DENSITY);
			saveCloud("cloudBeforeMoving.pcd", *cloudBeforeMoving);

			// Mientras se está moviendo el brazo, nadie debería poder obtener la nube a través del método getCloud
			gTransformation->cloudMutex.lock();
		}

	}

	void Arduino::armStoppedMoving()
	{
		if (IsFeatureMoveArmActive()) {
			unsigned int elapsedTime = (unsigned int) (ofGetSystemTime() - startTime);
			cout << "Brazo se dejo de mover luego de " << elapsedTime << "ms" << endl ;	

			armMoving = false;
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

			KEY_MOVE_1R = XML.getValue(ARDUINO_CONFIG "KEY_MOVE_1R", KEY_UNDEFINED).c_str()[0];
			KEY_MOVE_1L = XML.getValue(ARDUINO_CONFIG "KEY_MOVE_1L", KEY_UNDEFINED).c_str()[0];
			KEY_MOVE_2R = XML.getValue(ARDUINO_CONFIG "KEY_MOVE_2R", KEY_UNDEFINED).c_str()[0];
			KEY_MOVE_2L = XML.getValue(ARDUINO_CONFIG "KEY_MOVE_2L", KEY_UNDEFINED).c_str()[0];
			KEY_MOVE_4R = XML.getValue(ARDUINO_CONFIG "KEY_MOVE_4R", KEY_UNDEFINED).c_str()[0];
			KEY_MOVE_4L = XML.getValue(ARDUINO_CONFIG "KEY_MOVE_4L", KEY_UNDEFINED).c_str()[0];
			KEY_MOVE_8R = XML.getValue(ARDUINO_CONFIG "KEY_MOVE_8R", KEY_UNDEFINED).c_str()[0];
			KEY_MOVE_8L = XML.getValue(ARDUINO_CONFIG "KEY_MOVE_8L", KEY_UNDEFINED).c_str()[0];

			KEY_RESET = XML.getValue(ARDUINO_CONFIG "KEY_RESET", KEY_UNDEFINED).c_str()[0];
			KEY_PRINT_STATUS = XML.getValue(ARDUINO_CONFIG "KEY_PRINT_STATUS", KEY_UNDEFINED).c_str()[0];

			ANGLE_STEP = XML.getValue(ARDUINO_CONFIG "ANGLE_STEP", 2);

			ANGLE_STEP = XML.getValue(ARDUINO_CONFIG "HEIGHT", 2);
			ANGLE_STEP = XML.getValue(ARDUINO_CONFIG "LENGTH", 2);

			RESET_ANGLE1 = XML.getValue(ARDUINO_CONFIG "RESET_ANGLE1", ANGLE_DEFAULT);
			RESET_ANGLE2 = XML.getValue(ARDUINO_CONFIG "RESET_ANGLE2", ANGLE_DEFAULT);
			RESET_ANGLE4 = XML.getValue(ARDUINO_CONFIG "RESET_ANGLE4", ANGLE_DEFAULT);
			RESET_ANGLE8 = XML.getValue(ARDUINO_CONFIG "RESET_ANGLE8", ANGLE_DEFAULT);
			MOTOR_ANGLE_OFFSET	= XML.getValue(ARDUINO_CONFIG "MOTOR_ANGLE_OFFSET", MOTOR_ANGLE_OFFSET_DEFAULT);

			KINECT_HEIGHT = XML.getValue(ARDUINO_CONFIG "KINECT_HEIGHT", 0.0);
			MOTORS_HEIGHT = XML.getValue(ARDUINO_CONFIG "MOTORS_HEIGHT", 0.0);
			ARM_LENGTH = XML.getValue(ARDUINO_CONFIG "ARM_LENGTH", 0.0);

			MAX_ANGLE_1 = XML.getValue(ARDUINO_CONFIG "MAX_ANGLE_1", 0);
			MIN_ANGLE_1 = XML.getValue(ARDUINO_CONFIG "MIN_ANGLE_1", 0);
			MAX_ANGLE_2 = XML.getValue(ARDUINO_CONFIG "MAX_ANGLE_2", 0);
			MIN_ANGLE_2 = XML.getValue(ARDUINO_CONFIG "MIN_ANGLE_2", 0);
			MAX_ANGLE_4 = XML.getValue(ARDUINO_CONFIG "MAX_ANGLE_4", 0);
			MIN_ANGLE_4 = XML.getValue(ARDUINO_CONFIG "MIN_ANGLE_4", 0);
			MAX_ANGLE_8 = XML.getValue(ARDUINO_CONFIG "MAX_ANGLE_8", 0);
			MIN_ANGLE_8 = XML.getValue(ARDUINO_CONFIG "MIN_ANGLE_8", 0);

			ELAPSED_TIME_ARM_STOPPED_MOVING = XML.getValue(ARDUINO_CONFIG "ELAPSED_TIME_ARM_STOPPED_MOVING", 2000);
			ICP_CLOUD_DENSITY = XML.getValue(ARDUINO_CONFIG "ICP_CLOUD_DENSITY", 5);
			ICP_MAX_ITERATIONS = XML.getValue(ARDUINO_CONFIG "ICP_MAX_ITERATIONS", 20);
		}
	}

	ofVec3f Arduino::moveMotor(int motor_id, signed int degrees)
	{
		ofVec3f error = ofVec3f(INT_MAX, INT_MAX, INT_MAX);
		if (motor_id == ID_MOTOR_1)
		{
			if (!inRange(degrees, MIN_ANGLE_1, MAX_ANGLE_1))
			{
				return error;
			}
			else
			{
				angleMotor1 = degrees;
			}
		}
		if (motor_id == ID_MOTOR_2)
		{
			if (!inRange(degrees, MIN_ANGLE_2, MAX_ANGLE_2))
			{
				return error;
			}
			else
			{
				angleMotor2 = degrees;
			}
		}
		if (motor_id == ID_MOTOR_4)
		{
			if (!inRange(degrees, MIN_ANGLE_4, MAX_ANGLE_4))
			{
				return error;
			}
			else
			{
				angleMotor4 = degrees;
			}
		}
		if (motor_id == ID_MOTOR_8)
		{
			if (!inRange(degrees, MIN_ANGLE_8, MAX_ANGLE_8))
			{
				return error;
			}
			else
			{
				angleMotor8 = degrees;
			}
		}
		return getKinect3dCoordinates();
	}

}
