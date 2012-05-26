#include "vm.h"

#include "Feature.h"
#include "ofxXmlSettings.h"
#include "ofVec3f.h"
#include "winUtils.h"
#include "ofxOpenCv.h"

namespace mapinect {

#define		VM_CONFIG			"VMConfig:"

	static float nearPlane;
	static float farPlane;

	static float transX =  -0.088;	//0.019;
	static float transY =   0.004;	//-0.003;
	static float transZ =   0;		//-0.050;
	static float rotYAxis = 0;

	static float proj_matrix_glproj[16];
	static float proj_fx;
	static float proj_fy;
	static float proj_cx;
	static float proj_cy;

	static float proj_matrix_RT[16];
	static float inv_proj_RT_premult[16];

	static float kinect_matrix_RT[16];
	static float inv_kinect_RT_premult[16];


	static ofVec3f proj_loc, proj_fwd, proj_up, proj_trg;

	static bool bCustomFullscreen = false;

	std::string VM::proj_calib_file = "";
	std::string VM::kinect_calib_file = "";

	//--------------------------------------------------------------
	void VM::setup() {
		CHECK_ACTIVE;

		ofxXmlSettings XML;
		if(XML.loadFile("VM_Config.xml")) {

			nearPlane = XML.getValue(VM_CONFIG "NEAR_PLANE", 0.1);
			farPlane = XML.getValue(VM_CONFIG "FAR_PLANE", 20.0);

			VM::proj_calib_file = XML.getValue(VM_CONFIG "PROJ_CALIB", "data/calib/projector_calibration.yml");
			VM::kinect_calib_file = XML.getValue(VM_CONFIG "KINECT_CALIB", "data/calib/kinect_calibration.yml");

		}

 		loadProjCalibData(const_cast<char*>(proj_calib_file.c_str()));
		loadKinectExtrinsics(const_cast<char*>(kinect_calib_file.c_str()));		
//		keyPressed('0');
	}

	// Load calibration parameters for projector
	//		Camara Lucida, www.camara-lucida.com.ar
	void VM::loadProjCalibData(char* proj_calib_file) {
		
		CHECK_ACTIVE;

		// Load matrices from Projector's calibration file		
		CvMat* projector_intrinsics = (CvMat*) cvLoad(proj_calib_file, NULL, "proj_intrinsics");
		CvMat* projector_size = (CvMat*) cvLoad(proj_calib_file, NULL, "proj_size");
		CvMat* projector_R = (CvMat*) cvLoad(proj_calib_file, NULL, "R");
		CvMat* projector_T = (CvMat*) cvLoad(proj_calib_file, NULL, "T");

		// Obtain projector intrinsic parameters
		proj_fx = (float) cvGetReal2D(projector_intrinsics, 0, 0);
		proj_fy = (float) cvGetReal2D(projector_intrinsics, 1, 1);
		proj_cx = (float) cvGetReal2D(projector_intrinsics, 0, 2);
		proj_cy = (float) cvGetReal2D(projector_intrinsics, 1, 2);

		// Pruebo de modificar los parámetros intrínsecos para ver si se ajusta mejor el tamaño
		proj_fx = 2143.8887f;
		proj_fy = 1718.2029f;
//		proj_cx = 822.8236f;
//		proj_cy = 490.3096f;
		 
		float proj_width = (float) cvGetReal2D(projector_size, 0, 0);
		float proj_height = (float) cvGetReal2D(projector_size, 0, 1);

		// Create projection matrix for projector
		//		float proj_matrix_glproj[16];
		//	A	0	C	0
		//	0	B	D	0
		//	0	0	E	F
		//	0	0	-1	0

		float A = 2. * proj_fx  / proj_width;
		float B = 2. * proj_fy / proj_height;
		float C = 2. * (proj_cx / proj_width) - 1.;
		float D = 2. * (proj_cy / proj_height) - 1.;
		float E = - (farPlane + nearPlane) / (farPlane - nearPlane);
		float F = -2. * farPlane * nearPlane / (farPlane - nearPlane);
				
		proj_matrix_glproj[0]= A;				
		proj_matrix_glproj[1]= 0;					
		proj_matrix_glproj[2]= 0;			
		proj_matrix_glproj[3]= 0;				
		proj_matrix_glproj[4]= 0;
		proj_matrix_glproj[5]= B;
		proj_matrix_glproj[6]= 0;
		proj_matrix_glproj[7]= 0;
		proj_matrix_glproj[8]= C;
		proj_matrix_glproj[9]= D;
		proj_matrix_glproj[10]= E;
		proj_matrix_glproj[11]= -1;
		proj_matrix_glproj[12]= 0;
		proj_matrix_glproj[13]= 0;
		proj_matrix_glproj[14]= F;
		proj_matrix_glproj[15]= 0;

		// Create homogeneous transformation matrix (R|T) for Projector
		//R:
		//	xx	yx	zx	
		//	xy	yy	zy	
		//	xz	yz	zz
		//T:
		//	tx
		//	ty
		//	tz
		//RT:
		//		xx	yx	zx	tx	
		//		xy	yy	zy	ty
		//		xz	yz	zz	tz
		//		0	0	0	1
		// In OpenGl, matrices use col-major order, so RT should be stored as: {xx, xy, xz, 0, yx, yy, yz, 0, zx, zy, zz, 0, tx, ty, tz, 1} 		
		int j = 0;
		for(int i=0; i<3; i++){
			proj_matrix_RT[j]   = (float) cvGetReal2D(projector_R, 0, i);
			proj_matrix_RT[j+1] = (float) cvGetReal2D(projector_R, 1, i);
			proj_matrix_RT[j+2] = (float) cvGetReal2D(projector_R, 2, i);
			proj_matrix_RT[j+3] = 0;
			j+=4;
		}
		for(int i=0; i<3; i++){
			proj_matrix_RT[i+12] = (float) cvGetReal2D(projector_T,i,0);
		}
		// prueba
//		proj_matrix_RT[12] *= -1; // -tx Hay que invertirla esta componente
//		proj_matrix_RT[13] *= -1; // -ty Hay que invertirla esta componente
//		proj_matrix_RT[14] *= -1; // -tz No hay demasiada diferencia invirtiendo esta componente
		proj_matrix_RT[15] = 1;

		// Calculate inverse transformation with preMultTranslate
		ofVec3f proj_T((float) cvGetReal2D(projector_T,0,0), (float) cvGetReal2D(projector_T,1,0), (float) cvGetReal2D(projector_T,2,0));
		ofMatrix4x4 proj_R((float) cvGetReal2D(projector_R,0,0), (float) cvGetReal2D(projector_R,0,1), (float) cvGetReal2D(projector_R,0,2),0,
		 (float) cvGetReal2D(projector_R,1,0), (float) cvGetReal2D(projector_R,1,1), (float) cvGetReal2D(projector_R,1,2), 0,
		 (float) cvGetReal2D(projector_R,2,0), (float) cvGetReal2D(projector_R,2,1), (float) cvGetReal2D(projector_R,2,2), 0,
		 0,0,0,1);		
		ofMatrix4x4 inv_proj_RT = ofMatrix4x4(proj_R);
		inv_proj_RT.preMultTranslate(-proj_T);
		//inv_proj_RT = ofMatrix4x4::getTransposedOf(inv_proj_RT);
		//inv_proj_RT_premult = inv_proj_RT.getPtr(); // Devuelve la matriz en row-major order
		// No la traspongo para poder sacar fácil al float* en orden por columnas, como necesita opengl  
		j = 0;
		for(int i=0; i<4; i++){
			ofVec4f row_i = inv_proj_RT.getRowAsVec4f(i);
			inv_proj_RT_premult[j]   = row_i.x;
			inv_proj_RT_premult[j+1] = row_i.y;
			inv_proj_RT_premult[j+2] = row_i.z;
			inv_proj_RT_premult[j+3] = row_i.w;
			j+=4;
		}
		
		// Release matrices loaded with Projector's calib values 
		cvReleaseMat(&projector_intrinsics);
		cvReleaseMat(&projector_size);
		cvReleaseMat(&projector_R);
		cvReleaseMat(&projector_T); 
	}

	// Load kinect extrinsic parameters (matrices R and T)
	void VM::loadKinectExtrinsics(char* kinect_calib_file) {
		
		CHECK_ACTIVE;

		// Load matrices R and T from Kinect's calibration file		
		CvMat* kinect_R = (CvMat*) cvLoad(kinect_calib_file, NULL, "R");
		CvMat* kinect_T = (CvMat*) cvLoad(kinect_calib_file, NULL, "T");
		
		// Create homogeneous transformation matrix (R|T) for Kinect
		//R:
		//	xx	yx	zx	
		//	xy	yy	zy	
		//	xz	yz	zz
		//T:
		//	tx
		//	ty
		//	tz		
		//RT:
		//		xx	yx	zx	tx	
		//		xy	yy	zy	ty
		//		xz	yz	zz	tz
		//		0	0	0	1
		// In OpenGl, matrices use col-major order, so RT should be stored as: {xx, xy, xz, 0, yx, yy, yz, 0, zx, zy, zz, 0, tx, ty, tz, 1} 
		int j = 0;
		for(int i=0; i<3; i++){
			kinect_matrix_RT[j] = (float) cvGetReal2D(kinect_R, 0, i);
			kinect_matrix_RT[j+1] = (float) cvGetReal2D(kinect_R, 1, i);
			kinect_matrix_RT[j+2] = (float) cvGetReal2D(kinect_R, 2, i);
			kinect_matrix_RT[j+3] = 0;
			j+=4;
		}
		for(int i=0; i<3; i++){
			kinect_matrix_RT[i+12] = (float) cvGetReal2D(kinect_T,i,0);
		}
		kinect_matrix_RT[15] = 1;

		// Calculate inverse transformation
		//R:
		//	xx	xy	xz	0
		//	yx	yy	yz	0
		//	zx	zy	zz	0
		//	0	0	0	1
		//T:
		//	1	0	0	tx	
		//	0	1	0	ty
		//	0	0	1	tz
		//	0	0	0	1
		// (R|T) = T*R => Inv(T*R) = (T*R)^-1 = R^-1 * T^-1 = R^-1 * -T 
		//Inv(R|T):
		//		xx	yx	zx	-tx*xx -ty*yx -tz*zx	
		//		xy	yy	zy	-ty*xy -ty*yy -tz*zy
		//		xz	yz	zz	-tz*xz -ty*yz -tz*zz
		//		0	0	0	1
		// In OpenGl, matrices use col-major order, so RT should be stored as: {xx, yx, zx, 0, xy, yy, zy, 0, xz, yz, zz, 0, -tx, -ty, -tz, 1} 
		ofVec3f kin_T((float) cvGetReal2D(kinect_T,0,0), (float) cvGetReal2D(kinect_T,1,0), (float) cvGetReal2D(kinect_T,2,0));
		ofMatrix4x4 kin_R((float) cvGetReal2D(kinect_R,0,0), (float) cvGetReal2D(kinect_R,0,1), (float) cvGetReal2D(kinect_R,0,2),0,
		 (float) cvGetReal2D(kinect_R,1,0), (float) cvGetReal2D(kinect_R,1,1), (float) cvGetReal2D(kinect_R,1,2), 0,
		 (float) cvGetReal2D(kinect_R,2,0), (float) cvGetReal2D(kinect_R,2,1), (float) cvGetReal2D(kinect_R,2,2), 0,
		 0,0,0,1);		
		ofMatrix4x4 inv_kinect_RT = ofMatrix4x4(kin_R);
		inv_kinect_RT.preMultTranslate(-kin_T);
		//inv_kinect_RT = ofMatrix4x4::getTransposedOf(inv_kinect_RT);
		//inv_kinect_RT_premult = inv_kinect_RT.getPtr(); // Devuelve la matriz en row-major order
		// No la traspongo para poder sacar fácil al float* en orden por columnas, como necesita opengl  
		j = 0;
		for(int i=0; i<4; i++){
			ofVec4f row_i = inv_kinect_RT.getRowAsVec4f(i);
			inv_kinect_RT_premult[j]   = row_i.x;
			inv_kinect_RT_premult[j+1] = row_i.y;
			inv_kinect_RT_premult[j+2] = row_i.z;
			inv_kinect_RT_premult[j+3] = row_i.w;
			j+=4;
		}
		
		// Release matrices loaded with Kinect's calib values 
		cvReleaseMat(&kinect_R);
		cvReleaseMat(&kinect_T); 
	}

	//--------------------------------------------------------------
	void VM::setupView() {
	
		CHECK_ACTIVE;

/*		int w = ofGetWidth();
		int h = ofGetHeight();
		glViewport(0, 0, w, h);
*/
		glViewport(0, 0, 1280, 768);

		glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			glMultMatrixf(proj_matrix_glproj); 
			//gluPerspective(screenFov, aspect, nearPlane, farPlane);

/*		TRANSFORMACIONES ENTRE SISTEMAS DE COORDENADAS
		* Sist. de Coord. de cámara RGB (en RGBDemo)
			- Origen: cámara RGB
			- +X hacia izquierda (mirando de frente al Kinect)
			- +Y hacia abajo
			- +Z hacia donde uno está parado (mirando de frente al Kinect)
		* Sist. de Coord. de cámara de profundidad (en RGBDemo)
			- Origen: receptor IR? (Confirmar)
			- +X, +Y, +Z idem Sist. Coord. de cámara RGB
		* Sist. de Coord. de proyector (en RGBDemo)
			- Origen: proyector
			- +X, +Y, +Z idem Sist. Coord. de cámara RGB
		* Sist. de Coord. de OpenGL
			- +X hacia izquierda (mirando de frente al Kinect)
			- +Y hacia arriba
			- +Z hacia el Kinect (mirando de frente al Kinect)
		* Sist. de Coord. de ofxKinect
			- +X, +Y, +Z idem Sist. Coord. de cámara RGB

		=> Pasar del sist. de coord. de RGB a profundidad:
			Aplicar (R|T) del archivo de calibración del Kinect
		=> Pasar del sist. de coord. de profundidad a RGB:
			Aplicar transformación inversa de (R|T) (del archivo de calibración del Kinect). 
			Se sabe que: inv(R) = R^-1 = R^t (el inverso de la rotación es la matriz traspuesta)
		=> Pasar del sist. de coord. de RGB a proyector:
			Aplicar (R|T) del archivo de calibración del proyector
		=> Pasar del sist. de coord. de proyector al RGB:
			Aplicar transformación inversa de (R|T) (del archivo de calibración del proyector). 			
*/

		glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();
			glScalef(1,-1,-1);					//3 - Pasar del sist. de coord. del proyector al de OpenGL (invertir Y y Z)
			glTranslatef(transX,transY,transZ);
			//		glRotatef(rotYAxis,0,1,0);
			// R | -T
			glMultMatrixf(inv_proj_RT_premult);	//2 - Pasar de rgb-camera space al sistema de coordenadas del proyector (aplicar (R|T) del proyector)	
			// inv(R|T) = (T*R)^-1 = R^-1 * T^-1 = -R^-1*T
			glMultMatrixf(kinect_matrix_RT);	//1 - Pasar de depth-camera space a rgb-camera space (aplicar inverso de (R|T) del Kinect)		


/*		// Otra prueba:
		glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();
			glScalef(1,-1,-1);					//3 - Pasar del sist. de coord. del proyector al de OpenGL (invertir Y y Z)
			glMultMatrixf(proj_matrix_RT);		//2 - Pasar de rgb-camera space al sistema de coordenadas del proyector (aplicar (R|T) del proyector)
			//glMultMatrixf(inv_kinect_matrix_RT);//1 - Pasar de depth-camera space a rgb-camera space (aplicar inverso de (R|T) del Kinect)			glScalef(-1, -1, 1);			
			glMultMatrixf(inv_kinect_RT_premult);
*/
		// Hasta ahora
/*		glMatrixMode(GL_MODELVIEW);
//		ofPushMatrix();
			glLoadIdentity();
			glScalef(-1, 1, -1);			//3 - Pasar del espacio de coordenadas que creo es de Rgbdemo a OpenGL (rgbdemo.x = -opengl.x, rgbdemo.y = opengl.y, rgbdemo.z = -opengl.z)
			glMultMatrixf(proj_matrix_RT);	//2 - Aplicar rot y traslación del file de calibración del proyector
			glScalef(-1, -1, 1);			//1 - Pasar del espacio de coordenadas del ofxKinect al que creo es el de Rgbdemo (rgbdemo.x = -ofxKinect.x, rgbdemo.y = -ofxKinect.y, rgbdemo.z = ofxKinect.z)				
*/
/*			// Prueba 4
			glLoadIdentity();
			float initModelView [16] = {1,0,0,0, 0,-1,0,0, 0,0,-1,0, 0,0,0,1};
			glMultMatrixf(initModelView);
			glMultMatrixf(proj_matrix_RT);
*/
	}

	//--------------------------------------------------------------
	void VM::draw() {
		CHECK_ACTIVE;

	}

	//--------------------------------------------------------------
	void VM::endView() {
		CHECK_ACTIVE;
		
		//ofPopMatrix();
	}

	//--------------------------------------------------------------
	void VM::update() {
		CHECK_ACTIVE;
	}

	//--------------------------------------------------------------
	void VM::keyPressed(int key) {
		CHECK_ACTIVE;
				
		float varProj = 1.0f;

		float var = 0.01f;
		float varX = 0.001f;
		float varY = 0.001f;
		float varZ = 0.001f;

		switch (key) {
			/*********************
				  ADJUST VIEWING 
					 EYE    = (XPROJ,YPROJ,0)
					 LOOKAT = (XPROJ+TRANSXAT,
						YPROJ+TRANSYAT,
						-900.0)
			*********************
				YPROJ	= UP+
				YPROJ	= DOWN-
				XPROJ	= RIGHT+
				XPROJ	= LEFT-
				ZPROJ	= ++
				ZPROJ	= --
				TRANSXAT= l+
				TRANSXAT= j-
				TRANSYAT= i+
				TRANSYAT= k-
			********************/	
			case OF_KEY_UP:
				proj_fy +=varProj;
				printf("proj_fy increased: %f \n",proj_fy);
				setProjMatrix(proj_fx, proj_fy, proj_cx, proj_cy);
				break;
				/*transY +=varY;
				proj_loc.y += varY;
				proj_trg.y += varY;
				printf("proj_loc.y increased: %f \n", proj_loc.y);
				break; */
			case OF_KEY_DOWN:
				proj_fy -=varProj;
				printf("proj_fy decreased: %f \n",proj_fy);
				setProjMatrix(proj_fx, proj_fy, proj_cx, proj_cy);
				break;
				/*transY -=varY;
				proj_loc.y -= varY;
				proj_trg.y -= varY;
				printf("proj_loc.y decreased: %f \n", proj_loc.y);
				break;*/
			case OF_KEY_LEFT:
				proj_fx -=varProj;
				printf("proj_fx decreased: %f \n",proj_fx);
				setProjMatrix(proj_fx, proj_fy, proj_cx, proj_cy);
				break;
				/*transX -=varX;
				proj_loc.x -= varX;
				proj_trg.x -= varX;
				printf("proj_loc.x decreased: %f \n", proj_loc.x);
				break;*/
			case OF_KEY_RIGHT:
				proj_fx +=varProj;
				printf("proj_fx increased: %f \n",proj_fx);
				setProjMatrix(proj_fx, proj_fy, proj_cx, proj_cy);
				break;
				/*transX +=varX;
				proj_loc.x += varX;
				proj_trg.x += varX;
				printf("proj_loc.x increased: %f \n", proj_loc.x);
				break;*/
			case 'w':
/*				proj_cy +=varProj;
				printf("proj_cy increased: %f \n",proj_cy);
				setProjMatrix(proj_fx, proj_fy, proj_cx, proj_cy);
				break;
*/				transY += varY;
				printf("Y translation inc: %f \n", transY);
				break;
			case 's':
/*				proj_cy -=varProj;
				printf("proj_cy decreased: %f \n",proj_cy);
				setProjMatrix(proj_fx, proj_fy, proj_cx, proj_cy);
				break;
*/				transY -= varY;
				printf("Y translation dec: %f \n", transY);
				break;
			case 'a':
/*				proj_cx -=varProj;
				printf("proj_cx decreased: %f \n",proj_cx);
				setProjMatrix(proj_fx, proj_fy, proj_cx, proj_cy);
				break;
*/				transX -= varX;
				printf("X translation dec: %f \n", transX);
				break;
			case 'd':
/*				proj_cx +=varProj;
				printf("proj_cx increased: %f \n",proj_cx);
				setProjMatrix(proj_fx, proj_fy, proj_cx, proj_cy);
				break;
*/				transX += varX;
				printf("X translation inc: %f \n", transX);
				break;
			case '-':
				transZ -=varZ;
				proj_loc.z -= varZ;
				proj_trg.z -= varZ;
				printf("proj_loc.z decreased: %f \n", proj_loc.z);
				break;
			case '+':
				transZ +=varZ;
				proj_loc.z += varZ;
				proj_trg.z += varZ;
				printf("zProj increased: %f \n", proj_loc.z);
				break;

			/*********************
			  TOGGLE FULLSCREEN	 - 0 
			*********************/
			case '0':
				if (!bCustomFullscreen) 
				{
					ofBeginCustomFullscreen(1280, 0, 1280, 768);
					bCustomFullscreen = true;
				} else 
				{
					ofEndCustomFullscreen();
					bCustomFullscreen = false;
				}
				break;

			/*********************
			  SHOW STATUS	 - q
			*********************/
			case 'q':
				printf("Current viewing parameters: \n");
				printf("	Eye    = (%.6f,%.6f,%.6f) \n", proj_loc.x, proj_loc.y, proj_loc.z);
				printf("	LookAt = (%.6f,%.6f,%.6f) \n", proj_trg.x, proj_trg.y, proj_trg.z);
				break;

		case 'o':
			rotYAxis -= 1;
			//nearPlane += 0.005f;
			printf("near increased: %f \n",nearPlane);
			break;
		case 'p':
			rotYAxis += 1;
			//nearPlane -= 0.005f;
			printf("near increased: %f \n",nearPlane);
			break;
	/*	case ',':
			printf("Projection matrix:\n");
			for(int i=0;i<16;i=i+4){
				printf("[ %4.4f %4.4f %4.4f %4.4f] i=%d\n",projectionMatrix[i],projectionMatrix[i+1],projectionMatrix[i+2],projectionMatrix[i+3], i);
			}
			break;
	*/
		}
	}

	//--------------------------------------------------------------
	void VM::mouseMoved(int x, int y)
	{
		CHECK_ACTIVE;
	}

	//--------------------------------------------------------------
	void VM::mouseDragged(int x, int y, int button)
	{
		CHECK_ACTIVE;
	}

	//--------------------------------------------------------------
	void VM::mousePressed(int x, int y, int button)
	{
		CHECK_ACTIVE;
	}

	//--------------------------------------------------------------
	void VM::mouseReleased(int x, int y, int button)
	{
		CHECK_ACTIVE;
	}

	//--------------------------------------------------------------
	void VM::windowResized(int w, int h)
	{
		CHECK_ACTIVE;
	}

	//--------------------------------------------------------------
	bool VM::isActive()
	{
		return IsFeatureVMActive();
	}

	void VM::setProjMatrix(float fx, float fy, float cx, float cy){
		// Create projection matrix for projector
		//	A	0	C	0
		//	0	B	D	0
		//	0	0	E	F
		//	0	0	-1	0

		// fixed
		float w = 1280; 
		float h	= 768;

		float A = 2. * fx / w;
		float B = 2. * fy / h;
		float C = 2. * (cx / w) - 1.;
		float D = 2. * (cy / h) - 1.;

		proj_matrix_glproj[0]= A;				
		proj_matrix_glproj[5]= B;
		proj_matrix_glproj[8]= C;
		proj_matrix_glproj[9]= D;

	}

}
