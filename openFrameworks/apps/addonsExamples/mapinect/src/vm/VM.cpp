#include "vm.h"

#include "ofxXmlSettings.h"
#include "ofVec3f.h"
#include "winUtils.h"
#include "ofxOpenCv.h"

namespace mapinect {

#define		VM_CONFIG			"VMConfig:"

	static float screenFov;
	static float aspect;
	static float nearPlane;
	static float farPlane;

	static float xAngle;
	static float yAngle;
	static float zAngle;

	static float xTransLookAt;
	static float yTransLookAt;

	static float xProj;
	static float yProj;
	static float zProj; 

	static float proj_matrix_glproj[16];
	static ofVec3f proj_loc, proj_fwd, proj_up, proj_trg;

	static bool bCustomFullscreen = false;

	std::string VM::proj_calib_file = "";
	std::string VM::kinect_calib_file = "";

	//--------------------------------------------------------------
	void VM::setup() {
		ofxXmlSettings XML;
		if(XML.loadFile("VM_Config.xml")) {

			screenFov = XML.getValue(VM_CONFIG "SCREEN_FOV", 28.00f);
			aspect = XML.getValue(VM_CONFIG "ASPECT_RATIO", 1.33f);
			nearPlane = XML.getValue(VM_CONFIG "NEAR_PLANE", 0.1);
			farPlane = XML.getValue(VM_CONFIG "FAR_PLANE", 20.0);

			xAngle = XML.getValue(VM_CONFIG "ANGLE_X", 0.0f);
			yAngle = XML.getValue(VM_CONFIG "ANGLE_Y", 0.0f);
			zAngle = XML.getValue(VM_CONFIG "ANGLE_Z", 0.0f);

			xTransLookAt = XML.getValue(VM_CONFIG "TRANSLATION_LOOK_AT_X", 31.0f);
			yTransLookAt = XML.getValue(VM_CONFIG "TRANSLATION_LOOK_AT_Y", -9.0f);

			xProj = XML.getValue(VM_CONFIG "PROJECT_POS_X", 0.0f);
			yProj = XML.getValue(VM_CONFIG "PROJECT_POS_Y", 24.0f);
			zProj = XML.getValue(VM_CONFIG "PROJECT_POS_Z", 0.0f);

			VM::proj_calib_file = XML.getValue(VM_CONFIG "PROJ_CALIB", "data/calib/projector_calibration.yml");
			VM::kinect_calib_file = XML.getValue(VM_CONFIG "KINECT_CALIB", "data/calib/kinect_calibration.yml");

		}

		loadProjCalibData(const_cast<char*>(proj_calib_file.c_str()));
		
//		keyPressed('0');
	}

	// Load calibration parameters for projector
	//		Camara Lucida, www.camara-lucida.com.ar
	void VM::loadProjCalibData(char* proj_calib_file) {
	
		// Load matrices from Projector's calibration file		
		CvMat* projector_intrinsics = (CvMat*) cvLoad(proj_calib_file, NULL, "proj_intrinsics");
		CvMat* projector_size = (CvMat*) cvLoad(proj_calib_file, NULL, "proj_size");
		CvMat* projector_R = (CvMat*) cvLoad(proj_calib_file, NULL, "R");
		CvMat* projector_T = (CvMat*) cvLoad(proj_calib_file, NULL, "T");

		// Obtain projector intrinsic parameters
		float proj_fx = (float) cvGetReal2D(projector_intrinsics, 0, 0);
		float proj_fy = (float) cvGetReal2D(projector_intrinsics, 1, 1);
		float proj_cx = (float) cvGetReal2D(projector_intrinsics, 0, 2);
		float proj_cy = (float) cvGetReal2D(projector_intrinsics, 1, 2);
		 
		float proj_width = (float) cvGetReal2D(projector_size, 0, 0);
		float proj_height = (float) cvGetReal2D(projector_size, 0, 1);

		// Create projection matrix for projector
		//		float proj_matrix_glproj[16];
		//	A	0	C	0
		//	0	B	D	0
		//	0	0	E	F
		//	0	0	-1	0

		float A = 2. * proj_fx / proj_width;
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

		// Create viewing matrix for projector
		float proj_matrix_RT[16];
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
		int j = 0;
		for(int i=0; i<3; i++){
			proj_matrix_RT[j] = (float) cvGetReal2D(projector_R, 0, i);
			proj_matrix_RT[j+1] = (float) cvGetReal2D(projector_R, 1, i);
			proj_matrix_RT[j+2] = (float) cvGetReal2D(projector_R, 2, i);
			j+=4;
		}
		proj_matrix_RT[3] = 0;
		proj_matrix_RT[7] = 0;
		proj_matrix_RT[11] = 0;
		for(int i=0; i<3; i++){
			proj_matrix_RT[i+12] = (float) cvGetReal2D(projector_T,i,0);
		}
		proj_matrix_RT[15] = 1;

		// Set projector vectors for viewing
		proj_loc = ofVec3f(proj_matrix_RT[12], proj_matrix_RT[13], proj_matrix_RT[14]); // tx ty tz	
		proj_fwd = ofVec3f(proj_matrix_RT[8],  proj_matrix_RT[9],  proj_matrix_RT[10]); // zx zy zz
		proj_up =  ofVec3f(proj_matrix_RT[4],  proj_matrix_RT[5],  proj_matrix_RT[6]);  // yx yy yz
		proj_trg = proj_loc + proj_fwd;

		// Release matrices loaded with Projector's calib values 
		cvReleaseMat(&projector_intrinsics);
		cvReleaseMat(&projector_size);
		cvReleaseMat(&projector_R);
		cvReleaseMat(&projector_T); 

	}

	//--------------------------------------------------------------
	void VM::setupView() {
	
/*		int w = ofGetWidth();
		int h = ofGetHeight();
		glViewport(0, 0, w, h);
*/
		glViewport(0, 0, 1280, 768);

		glMatrixMode(GL_PROJECTION);
//		ofPushMatrix();
			glLoadIdentity();
			glMultMatrixf(proj_matrix_glproj); 
			//gluPerspective(screenFov, aspect, nearPlane, farPlane);
//		ofPopMatrix();

		glMatrixMode(GL_MODELVIEW);
//		ofPushMatrix();
			glLoadIdentity();
			glScalef(-1, -1, 1);	
			gluLookAt(proj_loc.x, proj_loc.y, proj_loc.z,	//loc
						proj_trg.x, proj_trg.y, proj_trg.z,	//target
						proj_up.x, proj_up.y, proj_up.z);	//up
//		ofPopMatrix();


	/*	ofPushMatrix();
			glLoadIdentity();

			ofVec3f eyePos (xProj, yProj, zProj);
			ofVec3f lookAtPos (xProj + xTransLookAt, yProj + yTransLookAt, -900.0);
			ofVec3f lookAt = lookAtPos - eyePos;
			ofVec3f right (1, 0, 0);
			ofVec3f upDir = right.cross(lookAt);
			gluLookAt(eyePos.x, eyePos.y, eyePos.z, lookAtPos.x, lookAtPos.y, lookAtPos.z, upDir.x, upDir.y, upDir.z);

			//glTranslatef(-180,0,0);
			//glTranslatef(transX,transY,0);
			glRotatef(zAngle, 0, 0, 1);
			glScalef(1000, -1000, -1000);
	*/
	}

	//--------------------------------------------------------------
	void VM::draw() {

	}

	//--------------------------------------------------------------
	void VM::endView() {
		ofPopMatrix();
	}

	//--------------------------------------------------------------
	void VM::update() {
	
	}

	//--------------------------------------------------------------
	void VM::keyPressed(int key) {
		float var = 0.01f;
		float varX = 1.0f;
		float varY = 1.0f;
		float varZ = 5.0f;

		switch (key) {
			/*********************
			  ADJUST FRUSTUM 
			*********************
				FOV_Y	= W+
				FOV_Y	= S-
				ASPECT	= A-
				ASPECT	= D+
			********************/	
			case 'w':
				screenFov += var;
				printf("screenFov increased: %f \n",screenFov);
				break;
			case 's':
				screenFov -= var;
				printf("screenFov decreased: %f \n",screenFov);
				break;
			case 'a':
				aspect +=var;
				printf("aspect increased: %f \n",aspect);
				break;
			case 'd':
				aspect -=var;
				printf("aspect increased: %f \n",aspect);
				break;

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
				yProj += varY;
				printf("yProj increased: %f \n",yProj);
				break;
			case OF_KEY_DOWN:
				yProj -= varY;
				printf("yProj decreased: %f \n",yProj);
				break;
			case OF_KEY_LEFT:
				xProj -= varX;
				printf("xProj decreased: %f \n",xProj);
				break;
			case OF_KEY_RIGHT:
				xProj += varX;
				printf("xProj increased: %f \n",xProj);
				break;
			case '-':
				zProj -= 10.0f;
				printf("zProj decreased: %f \n",zProj);
				break;
			case '+':
				zProj += 10.0f;
				printf("zProj increased: %f \n",zProj);
				break;
			case 'j':
				xTransLookAt += varX;
				printf("xTransLookAt increased: %f \n", xTransLookAt);
				break;
			case 'l':
				xTransLookAt -= varX;
				printf("xTransLookAt increased: %f \n", xTransLookAt);
				break;
			case 'i':
				yTransLookAt += varY;
				printf("yTransLookAt increased: %f \n", yTransLookAt);
				break;
			case 'k':
				yTransLookAt -= varY;
				printf("yTransLookAt increased: %f \n", yTransLookAt);
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
				printf("Current projection parameters: \n");
				printf("	aspect: %.4f \n	fov: %.4f \n", aspect, screenFov);
				printf("Current viewing parameters: \n");
				printf("	Eye    = (%.2f,%.2f,%.2f) \n", xProj, yProj, zProj);
				printf("	LookAt = (%.2f,%.2f,%.2f) \n", xProj + xTransLookAt, yProj + yTransLookAt, -900.0);
				break;

		case 'o':
			nearPlane += 0.005f;
			printf("near increased: %f \n",nearPlane);
			break;
		case 'p':
			nearPlane -= 0.005f;
			printf("near increased: %f \n",nearPlane);
			break;
	/*	case ',':
			printf("Projection matrix:\n");
			for(int i=0;i<16;i=i+4){
				printf("[ %4.4f %4.4f %4.4f %4.4f] i=%d\n",projectionMatrix[i],projectionMatrix[i+1],projectionMatrix[i+2],projectionMatrix[i+3], i);
			}
			break;
	*/	case 'm':
			zAngle += 1.0f;
			printf("Angle increased for rotation Z axis: %f \n",zAngle);
			break;
		case 'n':
			zAngle -= 1.0f;
			printf("Angle decreased for rotation Z axis: %f \n",zAngle);
			break;
		}
	}

	//--------------------------------------------------------------
	void VM::mouseMoved(int x, int y)
	{
	}

	//--------------------------------------------------------------
	void VM::mouseDragged(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void VM::mousePressed(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void VM::mouseReleased(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void VM::windowResized(int w, int h)
	{
	}

}
