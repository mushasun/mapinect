#include "vm.h"

#include "utils.h"
#include "winUtils.h"
#include "Model.h"
#include "PCPolyhedron.h"

namespace mapinect {

	float screenFov = 25.40f;		//28.04f;
	float aspect = 1.36f;			//1.35f;
	float nearPlane = 0.3f;
	float zAngle = 0.0f;

	float transXAT = -45.0;			//-59.0;		
	float transYAT = -6.0;		

	// Coordenadas 3D del proyector
	float xProj = -20.0f;		
	float yProj = 33.0f;		// 364 mm arriba del Kinect
	float zProj = 0.0f;			// 720 o 900 mm de distancia (z) del Kinect

	float projectionMatrix[16];

	// Dibujar Quad
	static ofxVec3f vA,vB,vC,vD;

	static ofxVec3f detectedQuads[12][4];

	GLuint VM::textureID = 0;
	unsigned char* VM::imgPixels = NULL;

	//--------------------------------------------------------------
	void VM::setup() {
		bImgLoaded = false;
//		textureID = loadImageTexture("ofTheo.jpg");
	}



	ofxVec3f scaleFromMtsToMms(ofxVec3f p)
	{
		ofxVec3f res;
		// Transform from meters to milimeters
		res.x = (p.x)*1000;
		res.y = (p.y)*1000;
		res.z = (p.z)*1000;
		return res;	
	}

	//--------------------------------------------------------------
	void VM::draw()
	{

		textureID = loadImageTexture("Building_texture.jpg");

		float nearDist 	= 300; //30 cms  //zProj / 10;	//This is also the viewing plane distance	
		float farDist 	= 4000; //zProj * 10.0;	//4.0

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(screenFov, aspect, nearDist, farDist);

		glGetFloatv(GL_PROJECTION_MATRIX, projectionMatrix);


		glMatrixMode(GL_MODELVIEW);
		ofPushMatrix();
		glLoadIdentity();

		ofxVec3f eyePos (xProj, yProj, zProj);
		ofxVec3f lookAtPos (xProj+transXAT, yProj+transYAT, -900.0);
		ofxVec3f lookAt = lookAtPos - eyePos;
		ofxVec3f right (1,0,0);
		ofxVec3f upDir = right.cross(lookAt);
		gluLookAt(eyePos.x, eyePos.y, eyePos.z, lookAtPos.x, lookAtPos.y, lookAtPos.z, upDir.x, upDir.y, upDir.z);

		//glTranslatef(-180,0,0);
		//glTranslatef(transX,transY,0);
		glRotatef(zAngle,0,0,1);

		// Obtener caras detectadas por el Kinect
		int cantQuads = 0;
		gModel->objectsMutex.lock();
		if (gModel->objects.size() > 0) 
		{
			for(list<mapinect::ModelObject*>::iterator k = gModel->objects.begin(); 
				k != gModel->objects.end(); k++)
			{
				PCPolyhedron* hedron = (PCPolyhedron*)(*k);
				for (int i=0; i<hedron->getPCPolygonSize();i++)
				{
					PCPolygon* gon = hedron->getPCPolygon(i);
					if (gon->hasObject()) 
					{
						mapinect::Polygon* q = gon->getPolygonModelObject();
						vA = q->getVertex(0);
						vB = q->getVertex(1);
						vC = q->getVertex(2);
						vD = q->getVertex(3);
						// Invert Y and Z due to difference between Kinect and world orientations
						vA.z = vA.z*-1;
						vB.z = vB.z*-1;
						vC.z = vC.z*-1;
						vD.z = vD.z*-1;
						vA.y = vA.y*-1;
						vB.y = vB.y*-1;
						vC.y = vC.y*-1;
						vD.y = vD.y*-1;
		
						//SCALE KINECT MEASURE (MTS) INTO MM
						vA = scaleFromMtsToMms(vA);
						vB = scaleFromMtsToMms(vB);
						vC = scaleFromMtsToMms(vC);
						vD = scaleFromMtsToMms(vD);

						detectedQuads[cantQuads][0] = vA;
						detectedQuads[cantQuads][1] = vB;
						detectedQuads[cantQuads][2] = vC;
						detectedQuads[cantQuads][3] = vD;
						cantQuads++;
					}			
				}					
			}
		} else {
			// Quad de prueba
			vA.set(-100,100,-500);
			vB.set(0,100,-500);
			vC.set(0,0,-500);
			vD.set(-100,0,-500);
			detectedQuads[cantQuads][0] = vA;
			detectedQuads[cantQuads][1] = vB;
			detectedQuads[cantQuads][2] = vC;
			detectedQuads[cantQuads][3] = vD;
			cantQuads++;
		}
		gModel->objectsMutex.unlock();
		
		// Draw each quad
		for(int i=0; i<cantQuads; i++) 
		{
			vA = detectedQuads[i][0];
			vB = detectedQuads[i][1];
			vC = detectedQuads[i][2];
			vD = detectedQuads[i][3];

//			ofSetColor(0xFF0000);
			
			// If image loaded correctly
			if (bImgLoaded) {
				// Bind Texture
				glEnable(GL_TEXTURE_2D);
				glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
				// GL_REPLACE can be specified to just draw the surface using the texture colors only
				// GL_MODULATE means the computed surface color is multiplied by the texture color (to be used when lighting)
				glBindTexture(GL_TEXTURE_2D, textureID); 

				// Draw quad and map 2D points for texture mapping
				glBegin(GL_QUADS);      
					glTexCoord2f(0, 0);
					glVertex3f(vA.x,vA.y,vA.z);    
					glTexCoord2f(1, 0);
					glVertex3f(vB.x,vB.y,vB.z);
					glTexCoord2f(1, 1);
					glVertex3f(vC.x,vC.y,vC.z);
					glTexCoord2f(0, 1);
					glVertex3f(vD.x,vD.y,vD.z);
				glEnd();

			} else {
				glDisable(GL_TEXTURE_2D);
				glBegin(GL_QUADS);      
					glVertex3f(vA.x,vA.y,vA.z);    
					glVertex3f(vB.x,vB.y,vB.z);
					glVertex3f(vC.x,vC.y,vC.z);
					glVertex3f(vD.x,vD.y,vD.z);
				glEnd();			
			}

		}

		ofPopMatrix();

//		glDeleteTextures(1,&textureID);
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
				transXAT += varX;
				printf("transXAT increased: %f \n",transXAT);
				break;
			case 'l':
				transXAT -= varX;
				printf("transXAT increased: %f \n",transXAT);
				break;
			case 'i':
				transYAT += varY;
				printf("transYAT increased: %f \n",transYAT);
				break;
			case 'k':
				transYAT -= varY;
				printf("transYAT increased: %f \n",transYAT);
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
				printf("	LookAt = (%.2f,%.2f,%.2f) \n", xProj+transXAT, yProj+transYAT, -900.0);
				printf("Current quad coordinates: \n");
				printf("	A = (%.4f,%.4f,%.4f) \n", vA.x, vA.y, vA.z);
				printf("	B = (%.4f,%.4f,%.4f) \n", vB.x, vB.y, vB.z);
				printf("	C = (%.4f,%.4f,%.4f) \n", vC.x, vC.y, vC.z);
				printf("	D = (%.4f,%.4f,%.4f) \n", vD.x, vD.y, vD.z);
				break;

		case 'o':
			nearPlane += 0.005f;
			printf("near increased: %f \n",nearPlane);
			break;
		case 'p':
			nearPlane -= 0.005f;
			printf("near increased: %f \n",nearPlane);
			break;
		case ',':
			printf("Projection matrix:\n");
			for(int i=0;i<16;i=i+4){
				printf("[ %4.4f %4.4f %4.4f %4.4f] i=%d\n",projectionMatrix[i],projectionMatrix[i+1],projectionMatrix[i+2],projectionMatrix[i+3], i);
			}
			break;
		case 'm':
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

	GLuint VM::loadImageTexture(char* imgFile)
	{
		if ((imgFile == NULL) || (imgFile == "")) {
			bImgLoaded = false;
			return -1;
		} else {
			bImgLoaded = img.loadImage(imgFile);
			if (bImgLoaded) {
				imgFilename = imgFile;
				imgPixels = img.getPixels();
				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
				glEnable(GL_TEXTURE_2D);
				glPixelStorei (GL_UNPACK_ALIGNMENT, 1);
				glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
				//glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
  				glGenTextures(1, &textureID);
				glBindTexture(GL_TEXTURE_2D, textureID);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);	
				glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA, img.getWidth(), img.getHeight(), 0, GL_RGB/*GL_RGBA*/,GL_UNSIGNED_BYTE,imgPixels);
				img.setUseTexture(true);
				glFlush();
				return textureID;
			} else {
				return -1;
			}
		}
	} 


}
