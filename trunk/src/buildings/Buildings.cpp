#include "buildings.h"

#include "utils.h"
#include "winUtils.h"
#include "Model.h"
#include "PCPolyhedron.h"

#define MAX_EDIF 10


namespace mapinect {



	//float projectionMatrix[16];
	//float projInt[9];

	// Dibujar Quad
	

	static ofxVec3f detectedQuads[12][4];

	//--------------------------------------------------------------
	void buildings::setup(ofxFenster* f) {
		//bImgLoaded = false;
		for (int i = 0; i<MAX_EDIF; i++)
		{
			porcentajes.push_back(0.0f);
		}
		this->fenster = f;	
		textureID = loadImageTexture("data/texturas/Building_texture.jpg");
		streetT = loadImageTexture("data/texturas/brickPath_2.jpg");
		camino = loadImageTexture("data/texturas/oba.jpg");

		screenFov = 28.00f;		//25.40f//28.04f;
		aspect = 1.33f;			//1.36f//1.35f;
		nearPlane = 0.3f;
		zAngle = 0.0f;


		transXAT = -24.0; //-11.0;//-45.0;			//-59.0;		
		transYAT = 26.0; //76.0;//-6.0;		

		// Coordenadas 3D del proyector
		xProj = 0;//-20.0f;		
		yProj = 30;//33.0f;		// 364 mm arriba del Kinect
		zProj = 0;//0.0f;			// 720 o 900 mm de distancia (z) del Kinect

	}

	//ofxVec3f scaleFromMtsToMms(ofxVec3f p)
	//{
	//	ofxVec3f res;
	//	// Transform from meters to milimeters
	//	res.x = (p.x)*1000;
	//	res.y = (p.y)*1000;
	//	res.z = (p.z)*1000;
	//	return res;	
	//}



	//--------------------------------------------------------------
	void buildings::draw()
	{
		
		//textureID = loadImageTexture("globe.jpg");

		float nearDist 	= 300; //30 cms  //zProj / 10;	//This is also the viewing plane distance	
		float farDist 	= 4000; //zProj * 10.0;	//4.0

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(screenFov, aspect, nearDist, farDist);
	
/*		// Prueba
		double w = ofGetWidth();
		double h = ofGetHeight();
		glOrtho(0,w,0,h,-1,1);

		projInt[0] = 900;	// alfa_x 
		projInt[1] = 0;		// skew
		projInt[2] = 600;	// u_0 - principal point x
		projInt[3] = 0;
		projInt[4] = 900;	// alfa_y
		projInt[5] = 700;	// v_0 - principal point y
		projInt[6] = 0;
		projInt[7] = 0;
		projInt[8] = 1;

		glMultMatrixf(projInt);
*/
		//glGetFloatv(GL_PROJECTION_MATRIX, projectionMatrix);


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

		std::vector<ofxVec3f> all_pts; //se va a usar después para dibujar las calles, va a tener todos lo puntos
									  //de las caras



		if(gModel->table != NULL)
		{
			//ofScale(3500,3500,3500);
			PCPolyhedron* mesa = (PCPolyhedron*)(gModel->table);
				for (int i=0; i<mesa->getPCPolygonSize();i++)
				{
					PCPolygon* gon = mesa->getPCPolygon(i);
					if (gon->hasObject()) 
					{
						mapinect::Polygon* q = gon->getPolygonModelObject();

						ofxVec3f vA,vB,vC,vD;

						vA = q->getVertex(0);
						vB = q->getVertex(1);
						vC = q->getVertex(2);
						vD = q->getVertex(3);
						//SCALE KINECT MEASURE (MTS) INTO MM
						vA = scaleFromMtsToMms(vA);
						vB = scaleFromMtsToMms(vB);
						vC = scaleFromMtsToMms(vC);
						vD = scaleFromMtsToMms(vD);

						vA.z = vA.z*-1;
						vB.z = vB.z*-1;
						vC.z = vC.z*-1;
						vD.z = vD.z*-1;
						vA.y = vA.y*-1;
						vB.y = vB.y*-1;
						vC.y = vC.y*-1;
						vD.y = vD.y*-1;
						

						glEnable(GL_TEXTURE_2D);
						glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
						// GL_REPLACE can be specified to just draw the surface using the texture colors only
						// GL_MODULATE means the computed surface color is multiplied by the texture color (to be used when lighting)
						glBindTexture(GL_TEXTURE_2D, streetT);	

						//ofSetColor(0xFF0000);
						glBegin(GL_QUADS);      
							glVertex3f(vA.x,vA.y,vA.z); 
							glTexCoord2f(1, 0);
							glVertex3f(vB.x,vB.y,vB.z);
							glTexCoord2f(0, 0);
							glVertex3f(vC.x,vC.y,vC.z);
							glTexCoord2f(1, 1);
							glVertex3f(vD.x,vD.y,vD.z);
							glTexCoord2f(0, 1);
						glEnd();

						//glBindTexture(GL_TEXTURE_2D, 0);

						/*ofSetColor(0,0,255);

						ofSetColor(255,255,255);
						glPointSize(10);
						glEnable(GL_POINT_SMOOTH);
						glBegin(GL_POINTS);
						ofSetColor(255,255,255);
						glVertex3f(vA.x,vA.y,vA.z);    
						ofSetColor(255,255,0);
						glVertex3f(vB.x,vB.y,vB.z);    
						ofSetColor(0,255,0);
						glVertex3f(vC.x,vC.y,vC.z);    
						ofSetColor(0,0,255);
						glVertex3f(vD.x,vD.y,vD.z);    
						glEnd();*/
						
					}			
				}
		}



		if (gModel->objects.size() > 0) 
		{
			std::vector<ofxVec3f> centroides;
			for(list<mapinect::ModelObject*>::iterator k = gModel->objects.begin(); 
				k != gModel->objects.end(); k++)
			{
				PCPolyhedron* hedron = (PCPolyhedron*)(*k);

				centroides.push_back(hedron->getCenter());

				for (int i=0; i<hedron->getPCPolygonSize();i++)
				{
					PCPolygon* gon = hedron->getPCPolygon(i);
					if (gon->hasObject()) 
					{
						mapinect::Polygon* q = gon->getPolygonModelObject();

						PCPolyhedron* mesa = (PCPolyhedron*)(gModel->table);
						ofxVec3f tableCenter = mesa->getCenter();
						ofxVec3f tableNormal = mesa->getPCPolygon(0)->getNormal().normalize();

						ofxVec3f caraNormal = gon->getNormal().normalize();

						float prod = abs(caraNormal.dot(tableNormal));
						int pid = hedron->getId();
						if (prod < 0.9)
						{

							

							static ofxVec3f vA,vB,vC,vD;

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

							std::vector<ofxVec3f> pts_;
			
							pts_.push_back(vA);
							pts_.push_back(vB);
							pts_.push_back(vC);
							pts_.push_back(vD);

							all_pts.push_back(vA);
							all_pts.push_back(vB);
							all_pts.push_back(vC);
							all_pts.push_back(vD);

							sort(pts_.begin(), pts_.end(), sortOnY);

							/*	__________
								|2		3|
								|		 |
								|		 |
								|1		0|
							*/

							std::vector<ofxVec3f> pts_b;
							pts_b.push_back(pts_[0]);
							pts_b.push_back(pts_[1]);

							sort(pts_b.begin(), pts_b.end(), sortOnX);

							std::vector<ofxVec3f> pts_a;
							pts_a.push_back(pts_[2]);
							pts_a.push_back(pts_[3]);

							sort(pts_a.begin(), pts_a.end(), sortOnX);

							ofxVec3f v0 = pts_b[1];
							ofxVec3f v1 = pts_b[0];
							ofxVec3f v2 = pts_a[0];
							ofxVec3f v3 = pts_a[1];

							float distancia12 = std::abs(v2.y - v1.y);
							float distancia03 = std::abs(v3.y - v0.y);			
			
							if (textureID>0) {
								// Bind Texture
								glEnable(GL_TEXTURE_2D);
								glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
								// GL_REPLACE can be specified to just draw the surface using the texture colors only
								// GL_MODULATE means the computed surface color is multiplied by the texture color (to be used when lighting)
								glBindTexture(GL_TEXTURE_2D, textureID); 

								// Draw quad and map 2D points for texture mapping

								glBegin(GL_QUADS);
									glTexCoord2f(0, 0);
									glVertex3f(v1.x, v1.y, v1.z);
									glTexCoord2f(porcentajes[pid], 0);
									glVertex3f(v0.x, v0.y, v0.z);
									glTexCoord2f(porcentajes[pid], porcentajes[pid]);
									glVertex3f(v3.x, v0.y + porcentajes[pid] * distancia03, v3.z);
									glTexCoord2f(0, porcentajes[pid]);
									glVertex3f(v2.x, v1.y + porcentajes[pid] * distancia12, v2.z);
								glEnd();

							} else {
								glDisable(GL_TEXTURE_2D);
								ofSetColor(0xFF0000);
								glBegin(GL_QUADS);      
									glVertex3f(v1.x, v1.y, v1.z);
									glVertex3f(v0.x, v0.y, v0.z);
									glVertex3f(v3.x, v0.y + porcentajes[pid] * distancia03, v3.z);
									glVertex3f(v2.x, v1.y + porcentajes[pid] * distancia12, v2.z);
								glEnd();			
							}


							if (porcentajes[pid] < 1.0f){
								porcentajes[pid] += 0.002f;
							}
						}
						else if (porcentajes[pid] > 0.99f) //(prod > 0.9)
							{
								static ofxVec3f vA,vB,vC,vD;

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

								std::vector<ofxVec3f> pts_;
			
								pts_.push_back(vA);
								pts_.push_back(vB);
								pts_.push_back(vC);
								pts_.push_back(vD);

								all_pts.push_back(vA);
								all_pts.push_back(vB);
								all_pts.push_back(vC);
								all_pts.push_back(vD);

								sort(pts_.begin(), pts_.end(), sortOnY);

								/*	__________
									|2		3|
									|		 |
									|		 |
									|1		0|
								*/

								std::vector<ofxVec3f> pts_b;
								pts_b.push_back(pts_[0]);
								pts_b.push_back(pts_[1]);

								sort(pts_b.begin(), pts_b.end(), sortOnX);

								std::vector<ofxVec3f> pts_a;
								pts_a.push_back(pts_[2]);
								pts_a.push_back(pts_[3]);

								sort(pts_a.begin(), pts_a.end(), sortOnX);

								ofxVec3f v0 = pts_b[1];
								ofxVec3f v1 = pts_b[0];
								ofxVec3f v2 = pts_a[0];
								ofxVec3f v3 = pts_a[1];

								float distancia12 = std::abs(v2.y - v1.y);
								float distancia03 = std::abs(v3.y - v0.y);			
			
								if (camino>0) {
									// Bind Texture
									glEnable(GL_TEXTURE_2D);
									glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
									// GL_REPLACE can be specified to just draw the surface using the texture colors only
									// GL_MODULATE means the computed surface color is multiplied by the texture color (to be used when lighting)
									glBindTexture(GL_TEXTURE_2D, camino); 

									// Draw quad and map 2D points for texture mapping

									glBegin(GL_QUADS);
										glTexCoord2f(0, 0);
										glVertex3f(v1.x, v1.y, v1.z);
										glTexCoord2f(porcentajes[pid], 0);
										glVertex3f(v0.x, v0.y, v0.z);
										glTexCoord2f(porcentajes[pid], porcentajes[pid]);
										glVertex3f(v3.x, v0.y + porcentajes[pid] * distancia03, v3.z);
										glTexCoord2f(0, porcentajes[pid]);
										glVertex3f(v2.x, v1.y + porcentajes[pid] * distancia12, v2.z);
									glEnd();

								} else {
									glDisable(GL_TEXTURE_2D);
									ofSetColor(0xFF0000);
									glBegin(GL_QUADS);      
										glVertex3f(v1.x, v1.y, v1.z);
										glVertex3f(v0.x, v0.y, v0.z);
										glVertex3f(v3.x, v0.y + porcentajes[pid] * distancia03, v3.z);
										glVertex3f(v2.x, v1.y + porcentajes[pid] * distancia12, v2.z);
									glEnd();			
								}
							}
						}
					}			
				}


				if (centroides.size() > 1)
				{
					//termine de procesar los objetos, tengo los centroides en centroides
					//a dibujar la calle
			
					//obtengo datos de la mesa
					PCPolyhedron* mesa = (PCPolyhedron*)(gModel->table);
					ofxVec3f tableCenter = mesa->getCenter();
					ofxVec3f tableNormal = mesa->getPCPolygon(0)->getNormal();

					//obtengo la proyeccion del primer punto
					ofxVec3f primero = centroides.back();
					ofxVec3f dif = primero - tableCenter;
					ofxVec3f proj = dif.dot(tableNormal) * tableNormal;
					primero = primero - proj;
					centroides.pop_back();

					//paso al segundo
					ofxVec3f segundo = centroides.back();
					dif = segundo - tableCenter;
					proj = dif.dot(tableNormal) * tableNormal;
					segundo = segundo - proj;
					centroides.pop_back();


					ofxVec3f distancia = primero - segundo;
					ofxVec3f vector_ancho = distancia.cross(tableNormal)/10;

					static ofxVec3f vA,vB,vC,vD;

					vA = primero + vector_ancho;
					vB = primero - vector_ancho;
					vC = segundo - vector_ancho;
					vD = segundo + vector_ancho;


					dif = vA - tableCenter;
					proj = dif.dot(tableNormal) * tableNormal;
					vA = vA - proj;

					dif = vB - tableCenter;
					proj = dif.dot(tableNormal) * tableNormal;
					vB = vB - proj;

					dif = vC - tableCenter;
					proj = dif.dot(tableNormal) * tableNormal;
					vC = vC - proj;

					dif = vD - tableCenter;
					proj = dif.dot(tableNormal) * tableNormal;
					vD = vD - proj;

					/*vA.z = vA.z*-1;
					vB.z = vB.z*-1;
					vC.z = vC.z*-1;
					vD.z = vD.z*-1;
					vA.y = vA.y*-1;
					vB.y = vB.y*-1;
					vC.y = vC.y*-1;
					vD.y = vD.y*-1;*/
		
					//SCALE KINECT MEASURE (MTS) INTO MM
					vA = scaleFromMtsToMms(vA);
					vB = scaleFromMtsToMms(vB);
					vC = scaleFromMtsToMms(vC);
					vD = scaleFromMtsToMms(vD);

					glEnable(GL_TEXTURE_2D);
					glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
					// GL_REPLACE can be specified to just draw the surface using the texture colors only
					// GL_MODULATE means the computed surface color is multiplied by the texture color (to be used when lighting)
					glBindTexture(GL_TEXTURE_2D, camino); 

					// Draw quad and map 2D points for texture mapping

					glBegin(GL_QUADS);
						glTexCoord2f(0, 0);
						glVertex3f(vA.x, vA.y, vA.z);
						glTexCoord2f(1, 0);
						glVertex3f(vB.x, vB.y, vB.z);
						glTexCoord2f(1, 1);
						glVertex3f(vC.x, vC.y, vC.z);
						glTexCoord2f(0, 1);
						glVertex3f(vD.x, vD.y, vD.z);
					glEnd();
				}
			

		} else {
			// Quad de prueba
			/*vA.set(-100,100,-500);
			vB.set(0,100,-500);
			vC.set(0,0,-500);
			vD.set(-100,0,-500);
			detectedQuads[cantQuads][0] = vA;
			detectedQuads[cantQuads][1] = vB;
			detectedQuads[cantQuads][2] = vC;
			detectedQuads[cantQuads][3] = vD;
			cantQuads++;*/
			/*vA.set(-60,0,-850);
			vB.set(-60,140,-850);
			vC.set(0,140,-800);
			vD.set(0,0,-800);
			detectedQuads[cantQuads][0] = vA;
			detectedQuads[cantQuads][1] = vB;
			detectedQuads[cantQuads][2] = vC;
			detectedQuads[cantQuads][3] = vD;
			cantQuads++;
			vA.set(-200,0,-650);
			vB.set(-200,160,-650);
			vC.set(-140,160,-755);
			vD.set(-140,0,-755);
			detectedQuads[cantQuads][0] = vA;
			detectedQuads[cantQuads][1] = vB;
			detectedQuads[cantQuads][2] = vC;
			detectedQuads[cantQuads][3] = vD;
			cantQuads++;*/
		}
		gModel->objectsMutex.unlock();
		

		ofPopMatrix();



	//	glDeleteTextures(1,&textureID);
	}


	//--------------------------------------------------------------
	void buildings::update() {
	
	}

	//--------------------------------------------------------------
	void buildings::keyPressed(int key) {
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
				/*printf("	A = (%.4f,%.4f,%.4f) \n", vA.x, vA.y, vA.z);
				printf("	B = (%.4f,%.4f,%.4f) \n", vB.x, vB.y, vB.z);
				printf("	C = (%.4f,%.4f,%.4f) \n", vC.x, vC.y, vC.z);
				printf("	D = (%.4f,%.4f,%.4f) \n", vD.x, vD.y, vD.z);*/
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
	void buildings::mouseMoved(int x, int y)
	{
	}

	//--------------------------------------------------------------
	void buildings::mouseDragged(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void buildings::mousePressed(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void buildings::mouseReleased(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void buildings::windowResized(int w, int h)
	{
	}

	GLuint buildings::loadImageTexture(char* imgFile)
	{
		GLuint result = -1;
		this->fenster->toContext();
		unsigned char* imgPixels;
		GLuint textureID;
		GLuint streetT;

		if ((imgFile == NULL) || (imgFile == "")) {
			bImgLoaded = false;
		} else {
			bImgLoaded = img.loadImage(imgFile);
			if (bImgLoaded) {
				imgFilename = imgFile;
				imgPixels = img.getPixels();
				//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
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
				result = textureID;
			}
		}

		this->fenster->toMainContext();
		return result;
	} 


}
