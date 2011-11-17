#include "vm.h"

#include "ofxXmlSettings.h"
#include "ofxVec3f.h"
#include "winUtils.h"

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


	static bool bCustomFullscreen = false;

	//--------------------------------------------------------------
	void VM::setup() {
		ofxXmlSettings XML;
		if(XML.loadFile("VM_Config.xml")) {

			screenFov = XML.getValue(VM_CONFIG "SCREEN_FOV", 28.00f);
			aspect = XML.getValue(VM_CONFIG "ASPECT_RATIO", 1.33f);
			nearPlane = XML.getValue(VM_CONFIG "NEAR_PLANE", 300.0f);
			farPlane = XML.getValue(VM_CONFIG "FAR_PLANE", 4000.0f);

			xAngle = XML.getValue(VM_CONFIG "ANGLE_X", 0.0f);
			yAngle = XML.getValue(VM_CONFIG "ANGLE_Y", 0.0f);
			zAngle = XML.getValue(VM_CONFIG "ANGLE_Z", 0.0f);

			xTransLookAt = XML.getValue(VM_CONFIG "TRANSLATION_LOOK_AT_X", 31.0f);
			yTransLookAt = XML.getValue(VM_CONFIG "TRANSLATION_LOOK_AT_Y", -9.0f);

			xProj = XML.getValue(VM_CONFIG "PROJECT_POS_X", 0.0f);
			yProj = XML.getValue(VM_CONFIG "PROJECT_POS_Y", 24.0f);
			zProj = XML.getValue(VM_CONFIG "PROJECT_POS_Z", 0.0f);

		}

	}

	//--------------------------------------------------------------
	void VM::setupView() {
		
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(screenFov, aspect, nearPlane, farPlane);
	
		glMatrixMode(GL_MODELVIEW);
		ofPushMatrix();
			glLoadIdentity();

			ofxVec3f eyePos (xProj, yProj, zProj);
			ofxVec3f lookAtPos (xProj + xTransLookAt, yProj + yTransLookAt, -900.0);
			ofxVec3f lookAt = lookAtPos - eyePos;
			ofxVec3f right (1, 0, 0);
			ofxVec3f upDir = right.cross(lookAt);
			gluLookAt(eyePos.x, eyePos.y, eyePos.z, lookAtPos.x, lookAtPos.y, lookAtPos.z, upDir.x, upDir.y, upDir.z);

			//glTranslatef(-180,0,0);
			//glTranslatef(transX,transY,0);
			glRotatef(zAngle, 0, 0, 1);
			glScalef(1000, -1000, -1000);
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
