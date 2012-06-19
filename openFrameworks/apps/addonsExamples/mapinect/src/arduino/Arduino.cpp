#include "Arduino.h"

#include "Feature.h"
#include "ofxXmlSettings.h"
#include <direct.h> // for getcwd
#include "Plane3D.h"
#include "utils.h"

#define M_PI_2     1.57079632679489661923

namespace mapinect {

#define		ARDUINO_CONFIG		"ArduinoConfig:"

#define		ID_MOTOR_1			1
#define		ID_MOTOR_2			2
#define		ID_MOTOR_4			4
#define		ID_MOTOR_8			8

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
	static int		ANGLE_STEP;
	static float	ARM_LENGTH;
	static float	KINECT_HEIGHT;
	static float	MOTORS_HEIGHT;
	static float	MOTOR_ANGLE_OFFSET;

	Arduino::Arduino()
	{
		angleMotor1 = 0;
		angleMotor2 = 0;
		angleMotor4 = 0;
		angleMotor8 = 0;  
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
		}
		
		angleMotor1 = RESET_ANGLE1;
		angleMotor2 = RESET_ANGLE2;
		angleMotor4 = RESET_ANGLE4;
		angleMotor8 = RESET_ANGLE8;
		//angleMotor8 = 90; // La posición inicial de este motor es mirando de costado. 
		if (serial.setup(COM_PORT, 9600)) {
			sendMotor((char) angleMotor1, ID_MOTOR_1);
			sendMotor((char) angleMotor2, ID_MOTOR_2);
			sendMotor((char) angleMotor4, ID_MOTOR_4);
			sendMotor((char) angleMotor8, ID_MOTOR_8);
		}

		//mira = ofVec3f(1, 0, 0);
		//posicion = ofVec3f(ARM_LENGTH, 0, 0);

		posicion = getKinect3dCoordinates();
		mira = lookingAt();

		//mira = ofVec3f(ARM_LENGTH, - KINECT_HEIGHT - MOTORS_HEIGHT, 1.0);
		//posicion = ofVec3f(ARM_LENGTH, - KINECT_HEIGHT - MOTORS_HEIGHT, 0.0);

		serial.enumerateDevices();
		
//		if (serial.setup(COM_PORT, 9600)) {
			//NO DEBERIA ESTAR ACA
//			lookAt(ofVec3f(0.35, -0.03, 0.10));
//			setKinect3dCoordinates(posicion);
		
//			return true;
	//	}

		return false;
	}

	void Arduino::exit() {
		CHECK_ACTIVE;
		if (serial.available()){
			serial.close();
		}
	}

	void Arduino::update() {
		CHECK_ACTIVE;
	}

	void Arduino::draw() {
		CHECK_ACTIVE;
	}

	void Arduino::keyPressed (int key) {
		CHECK_ACTIVE;

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
			ofVec3f cualquiera = ofVec3f(ARM_LENGTH, 0, 0.15);
			setKinect3dCoordinates(cualquiera);
		}
		else if (key == 'x')
		{
			lookAt(ofVec3f(0.35, 0.0, 0.10));
		}
		else if (key == 'c')
		{
			lookAt(ofVec3f(0.35, -0.03, 0.10));
		}
	}


	void Arduino::reset()
	{
		CHECK_ACTIVE;
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
		kinectPos = getWorldTransformation() * kinectPos;
		return ofVec3f(kinectPos.x(), kinectPos.y(), kinectPos.z());		
		//return posicion;	

/*		//angleMotor1 = motor de la base
		//angleMotor2 = motor del medio
		//angleMotor4 = motor de la punta
		//angleMotor8 = motor de mas de la punta
		//vamos a hallar las coordenadas x, y, z desde la base del brazo que está apoyada
		//en el la mesa.
		//hallemos z
		//sen(alpha) * el largo del brazo (la mitad desde el centro)
		double y = sin((float)angleMotor2) * ARM_LENGTH;
		double z = cos((float)angleMotor2) * ARM_LENGTH;
		double x = sin((float)angleMotor1) * y;
		ofVec3f position = ofVec3f(float(x), float (y), float (z));
		return position;
*/	}

	void Arduino::setKinect3dCoordinates(float x, float y, float z)
	{
		angleMotor2 = round(atan(z/x) * 180 / M_PI);			//el de la base, x no deberia ser 0 nunca
		if (y != 0) {
			if (y > 0)
			{
				angleMotor1 = (int)round(asin(-y/ARM_LENGTH) * 180 / M_PI); // estaba mal, era el asin
			}
			else
			{
				angleMotor1 = -(int)round(asin(y/ARM_LENGTH) * 180 / M_PI); // estaba mal, era el asin
			}
		}
		sendMotor((char) angleMotor1, ID_MOTOR_1);
		sendMotor((char) angleMotor2, ID_MOTOR_2);
		cout << read() << endl;
		cout << endl;
		posicion = ofVec3f(x, y, z);
	}

	ofVec3f Arduino::setKinect3dCoordinates(ofVec3f position)
	{
		//wrapper para posicionar desde un ofVec3f
		ofVec3f closest_position = find_closest_point_to_sphere(position);
		setKinect3dCoordinates(closest_position.x, closest_position.y, closest_position.z);
		//lookAt(mira_actual);
		return closest_position;
	}

	ofVec3f Arduino::convert_3D_cart_to_spher(ofVec3f cart_point)
	{
		//devuelve un ofVec3f que por simplicidad:
		//x = r, y = phi, z = theta
		//http://www.thecubiclewarrior.com/post/5954842175/spherical-to-cartesian
		float r = sqrt(pow(cart_point.x, 2) + pow(cart_point.y, 2) + pow(cart_point.z, 2) );
		float phi = acos(cart_point.y/r); //z es mi y! es el angulo en el plano x, z
		float theta = atan2(cart_point.x, cart_point.z);//z es mi y!
		ofVec3f spher_point(r, phi, theta);
		return spher_point;
	}

	ofVec3f Arduino::convert_3D_spher_to_cart(ofVec3f spher_point)
	{
		//http://www.thecubiclewarrior.com/post/5954842175/spherical-to-cartesian
		float r = spher_point.x;
		float phi = spher_point.y;
		float theta = spher_point.z;

		float z = r*sin(phi)*cos(theta);
		float x = r*sin(phi)*sin(theta);
		float y = r*cos(phi);

		ofVec3f cart_point(x, y, z);
		return cart_point;
	}

	ofVec3f Arduino::find_closest_point_to_sphere(ofVec3f point)
	{
		//la lógica es pasar el punto que viene a un punto en coordenadas esféricas
		//reducir el r y pasarlo nuevamente a coordenadas cartesianas.
		ofVec3f spher_orig_point = convert_3D_cart_to_spher(point);
		spher_orig_point.x = ARM_LENGTH;
		return convert_3D_spher_to_cart(spher_orig_point);
	}

	ofVec3f	Arduino::lookAt(ofVec3f point)
	{
		//TODO: tener el cuenta la traslacion del grueso de los motores de la punta
		//posicion = donde se encuentra ubicado
		//mira = donde estoy mirando ATM
		Eigen::Vector3f axisY (0, 1, 0);
		Eigen::Affine3f rotationY;
		float angleMotor2Rad = ofDegToRad(angleMotor2); // Motor de abajo del brazo, con la varilla "vertical"
		rotationY = Eigen::AngleAxis<float>(-angleMotor2Rad, axisY);

		Eigen::Vector3f axisX (1, 0, 0);
		Eigen::Affine3f rotationX;
		float angleMotor1Rad = ofDegToRad(angleMotor1);	// Motor que mueve la varilla "horizontal"
		rotationX = Eigen::AngleAxis<float>(-angleMotor1Rad, axisX);

		Eigen::Affine3f composed_matrix;
		composed_matrix = rotationY * rotationX;
		
		Eigen::Vector3f e_point = Eigen::Vector3f(point.x, point.y, point.z);
		Eigen::Vector3f e_mira = Eigen::Vector3f(mira.x, mira.y, mira.z);
		Eigen::Vector3f e_posicion = Eigen::Vector3f(posicion.x, posicion.y, posicion.z);

		Eigen::Vector3f et_point = composed_matrix * e_point;
		Eigen::Vector3f et_mira = composed_matrix * e_mira;
		Eigen::Vector3f et_posicion = composed_matrix * e_posicion;

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
		
		// Para monitorear los valores

		ofVec3f h_normal = horizontal.getNormal();
/*		*****FOR DEBUG*****		*/
/*		float hv_mira_len = hv_mira.length();
		float hv_point_len = hv_point.length();
		float hv_mira_dot = hv_mira.dot(h_normal);
		float h_normal_len = h_normal.length();
		float hv_point_dot = hv_point.dot(h_normal);
*/		//
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

		ofVec3f mira_trans = ofVec3f(t_mira.x - posicion.x, t_mira.y - posicion.y, t_mira.z - posicion.z); 
		mira_trans = mira_trans.rotate(-angulo_h, eje_y);
		t_mira = ofVec3f(mira_trans.x + posicion.x, mira_trans.y + posicion.y, mira_trans.z + posicion.z);

		ofVec3f pvt_point = vertical.project(t_point);
		ofVec3f pvt_mira = vertical.project(t_mira);
		ofVec3f pvt_posicion = vertical.project(t_posicion);

		ofVec3f vv_mira = pvt_mira - pvt_posicion;
		ofVec3f vv_point = pvt_point - pvt_posicion;

		float angulo_v = 0;
		ofVec3f v_normal = vertical.getNormal();
/*		*****FOR DEBUG*****		*/
/*		float vv_mira_len = vv_mira.length();
		float vv_point_len = vv_point.length();
		float vv_mira_dot = vv_mira.dot(v_normal);
		float v_normal_len = v_normal.length();
		float vv_point_dot = vv_point.dot(v_normal);
*/		//
		if (!(abs(vv_mira.length()) <= MATH_EPSILON || abs(vv_point.length()) <= MATH_EPSILON	
				|| (abs(vv_mira.dot(v_normal) - vv_mira.length()*v_normal.length()) <= MATH_EPSILON)
				|| (abs(vv_point.dot(v_normal) - vv_point.length()*v_normal.length()) <= MATH_EPSILON) )) {
				//if (length de alguno de los vectores < MATH_EPSILON || alguno de los vectores es "casi" paralelo a la normal del plano)	
				angulo_v = vv_mira.angle(vv_point);//motor 4
				if (point.y < 0)
				{
					angulo_v *= -1;
				}
		}		 	

		angleMotor4 = angulo_v;
		angleMotor8 = angulo_h;

		//mira_actual = point;
		mira = point;

		sendMotor((char) angulo_v, ID_MOTOR_4);
		sendMotor((char) angulo_h, ID_MOTOR_8);

		return NULL;
	}

	bool Arduino::isActive()
	{
		return IsFeatureArduinoActive();
	}
	ofVec3f	Arduino::lookingAt()
	{
		Eigen::Vector3f kinectMira (0.0, 0.0, 1.0);		// Mira inicial, en Sist. de Coord Local del Kinect
		kinectMira = getWorldTransformation() * kinectMira;
		return ofVec3f(kinectMira.x(), kinectMira.y(), kinectMira.z());	
		//return NULL;
		//return mira;
	}

	Eigen::Affine3f Arduino::getWorldTransformation()
	{
		float angleMotor1Rad = ofDegToRad(angleMotor1);	// Motor que mueve la varilla "horizontal"
		float angleMotor2Rad = ofDegToRad(angleMotor2); // Motor de abajo del brazo, con la varilla "vertical"
		float angleMotor4Rad = ofDegToRad(angleMotor4); // Motor de los de la punta, el de más arriba, sobre el que está enganchado la base del Kinect
		float angleMotor8Rad = ofDegToRad(angleMotor8 - 90); // Motor de los de la punta, el de más abajo. La posición inicial de referencia será 90 grados.

		//todas las matrices segun: http://pages.cs.brandeis.edu/~cs155/Lecture_07_6.pdf
		//CvMat* mat = cvCreateMat(4,4,CV_32FC1);
		//primero giramos según el eje vertical (Y)
		//el angulo es el del motor inferior y la matriz de transformación es:
		//[cos -sin  0  0]
		//[sen  cos  0  0]
		//[ 0    0   1  0]
		//[ 0    0   0  1]		// Coment Vero: Ojo que esta matriz es una rotación según el eje Z, no el Y

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
		
		Eigen::Affine3f composed_matrix;
		composed_matrix = rotationY * (rotationZ *  (translationX * (rotationY2 * (translationY * (rotationX * translationY2)))));
		//composed_matrix = translationY2 * rotationX * translationY * rotationY2 * translationX * rotationZ * rotationY;

		/*
			Pruebas para verificar que se estén calculando bien las transformaciones
		*/

		/* Las siguientes pruebas utilizan los valores de ángulos:
				angleMotor1 = -45;
				angleMotor2 = 0;
				angleMotor4 = 0;
				angleMotor8 = 90;		*/

		//Eigen::Vector3f ejemplo (0.0, 0.0, 0.0); //En Sist de Coordenadas Local, del Kinect
		// Debería dar como resultado: X = 0.14, Y = -0.35, Z = 0	=> Dio OK

		// Eigen::Vector3f ejemplo (0.0, MOTORS_HEIGHT + KINECT_HEIGHT, 0.0); //En Sist de Coordenadas Local, del Kinect
		// Debería dar como resultado: X = 0.25, Y = -0.25, Z = 0	=> Dio OK

		/* Las siguientes pruebas utilizan los valores de ángulos:
				angleMotor1 = -45;
				angleMotor2 = 0;
				angleMotor4 = -45;
				angleMotor8 = 90;		*/

		//Eigen::Vector3f ejemplo (0.0, 0.0, 0.0); //En Sist de Coordenadas Local, del Kinect
		// Dio como resultado: X = 0.14, Y = -0.33, Z = 0.07	=> Parece OK

		//Eigen::Vector3f ejemplo (0.1, 0.1, 0.1); //En Sist de Coordenadas Local, del Kinect
		// Dio como resultado: X = 0.32, Y = -0.30, Z = 0.07	=> Parece razonable...

		/* Las siguientes pruebas utilizan los valores de ángulos:
				angleMotor1 = 0;
				angleMotor2 = 0;
				angleMotor4 = -45;
				angleMotor8 = 45;		*/

		//Eigen::Vector3f ejemplo (0.0, 0.0, 0.0); //En Sist de Coordenadas Local, del Kinect
		// Dio como resultado: X = 0.40, Y = -0.13, Z = 0.05	=> Parece OK

		/* Las siguientes pruebas utilizan los valores de ángulos:
				angleMotor1 = 0;
				angleMotor2 = 45;
				angleMotor4 = 0;
				angleMotor8 = 90;		*/

		//Eigen::Vector3f ejemplo (0.0, 0.0, 0.0); //En Sist de Coordenadas Local, del Kinect
		// Dio como resultado: X = 0.24, Y = -0.16, Z = 0.24	=> Parece OK

		/*
		ejemplo = translationY2 * ejemplo;
		ejemplo = rotationX * ejemplo;
		ejemplo = translationY * ejemplo;
		ejemplo = rotationY2 * ejemplo;
		ejemplo = translationX * ejemplo;
		ejemplo = rotationZ * ejemplo;
		ejemplo = rotationY * ejemplo; 

		Eigen::Vector3f ej (0.0, 0.0, 0.0);
		ej = composed_matrix * ej;
		*/

		return composed_matrix;

	}
}
