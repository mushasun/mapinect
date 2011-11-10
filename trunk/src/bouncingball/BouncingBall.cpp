#include "BouncingBall.h"

#include "utils.h"
#include "winUtils.h"
#include "Model.h"
#include "PCPolyhedron.h"

namespace mapinect {
vector<ofxVec3f> unifyVertex(vector<ofxVec3f> lst, ofxVec3f& centroid)
	{
		centroid = ofxVec3f();
		vector<ofxVec3f> retList;
		bool unified = false;
		for(int i = 0; i < lst.size(); i++)
		{
			ofxVec3f pto = lst.at(i);
			for(int j = 0; j < retList.size(); j++)
			{
				if(pto.distance(retList.at(j)) < MAX_UNIFYING_DISTANCE_PROJECTION)
					unified = true;
			}
			if(!unified)
				retList.push_back(pto);
			unified = false;
		}

		for(int j = 0; j < retList.size(); j++)
		{
			centroid += retList.at(j);
		}

		centroid /= retList.size();
		return retList;
	}



	//--------------------------------------------------------------
	void BouncingBall::setup() {
		screenFov = 28.00f;		//25.40f//28.04f;
		aspect = 1.33f;			//1.36f//1.35f;
		nearPlane = 0.3f;
		zAngle = 0.0f;

		transXAT = 31.0; //-11.0;//-45.0;			//-59.0;		
		transYAT = -9.0; //76.0;//-6.0;		

		// Coordenadas 3D del proyector
		xProj = 0;//-20.0f;		
		yProj = 24;//33.0f;		// 364 mm arriba del Kinect
		zProj = 0;//0.0f;			// 720 o 900 mm de distancia (z) del Kinect


		y_angle = 0.0;
		tableSetted = false;

	}

	//--------------------------------------------------------------
	void BouncingBall::draw()
	{
		
		float nearDist 	= 300; //30 cms  //zProj / 10;	//This is also the viewing plane distance	
		float farDist 	= 4000; //zProj * 10.0;	//4.0

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(screenFov, aspect, nearDist, farDist);

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
		glRotatef(zAngle,0,0,1);

		if(tableSetted)
		{
			table->draw();
			

			//glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
			ball.draw();
			for (int j = 0; j < bobjects.size();j++)
				if(bobjects.at(j)->getId() != -1) //No es la mesa
					bobjects.at(j)->draw();
			//glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
			
		}
		ofPopMatrix();

	}

	vector<ofxVec3f> BouncingBall::closerToTable(vector<ofxVec3f> lst)
	{
		vector<ofxVec3f> lstret;
		lstret.push_back(lst.at(0));
		for(int i = 1; i < lst.size(); i++)
		{
			ofxVec3f curPto = lst.at(i);
			bool inserted = false;
			for (std::vector<ofxVec3f>::const_iterator it = lstret.begin (); it != lstret.end (); ++it)
			{
				if((curPto - tableCenter).length() > (*it -tableCenter).length())
				{
					//it--;
					lstret.insert(it,curPto);
					inserted = true;
					break;
				}
			}
			if(!inserted)
				lstret.push_back(curPto);
		}

		return lstret;
	}

	
	void closestPoints(vector<ofxVec3f> lst, ofxVec3f center, ofxVec3f& pt1,  ofxVec3f& pt2)
	{
		float dist1 = 9999;
		float dist2 = 9999;

		for(int i = 0; i < lst.size(); i++)
		{
			ofxVec3f pto = lst.at(i);
			if((pto - center).length() < dist1)
			{
				dist1 = (pto - center).length();
			}
		}
	}

	void BouncingBall::clearUnvisitedObjects()
	{
		std::vector<BObject*> keep;
		for (std::vector<BObject*>::const_iterator it = bobjects.begin (); it != bobjects.end (); ++it)
		{
			if((*it)->visited || (*it)->getId() == -1)
				keep.push_back(*it);
			else
				delete(*it);
		}
		bobjects = keep;
	}
	
	BObject* BouncingBall::getBObject(int id)
	{
		for(int i = 0; i < bobjects.size(); i++)
		{
			if(bobjects.at(i)->getId() == id)
			{
				bobjects.at(i)->visited = true;
				return bobjects.at(i);
			}
		}
		vector<Segment3D> nuVec;
		BObject* nuObject = new BObject(nuVec,ofxVec3f(rand()%256,rand()%256,rand()%256),id,rand()%6);
		bobjects.push_back(nuObject);
		return nuObject;
	}
	//--------------------------------------------------------------
	void BouncingBall::update() {

		gModel->objectsMutex.lock();
		if(!tableSetted)
		{
			if(gModel->table != NULL)
			{
				PCPolyhedron* hedron = (PCPolyhedron*)(gModel->table);
				PCPolygon* gon = hedron->getPCPolygon(0);
				if (gon->hasObject()) 
				{
					Polygon* q = gon->getPolygonModelObject();
					vA = q->getVertex(0);
					vB = q->getVertex(1);
					vC = q->getVertex(2);
					vD = q->getVertex(3);
					
					ofxVec3f center = hedron->getCenter();
					ofxVec3f w = ((vA - vC).getCrossed(vA - vD)).normalize();//gon->getNormal();
					tableNormal = w;
					tableCenter = center;

					ofxVec3f ballDir = w;
					ballDir.cross(vC - vA);
					ballDir.rotate(10,w);
					ballDir *= -1;
						//-(w.cross(vC - vA)).rotate(10,w);
					
					ball = Tejo(center,0.01,ballDir,0.005,w,tableCenter);
					Segment3D s1 = Segment3D(vA,vB,w,center);
					Segment3D s2 = Segment3D(vB,vC,w,center);
					Segment3D s3 = Segment3D(vC,vD,w,center);
					Segment3D s4 = Segment3D(vD,vA,w,center);
					
					tableSegment3Ds.push_back(s1);
					tableSegment3Ds.push_back(s2);
					tableSegment3Ds.push_back(s3);
					tableSegment3Ds.push_back(s4);
					table = new BObject(tableSegment3Ds, ofxVec3f(255,255,255), -1,0);
					table->setPolyhedron(hedron);
					//segments.push_back(s5);
					//segments.push_back(s6);
					bobjects.push_back(table);
					tableSetted = true;
				}	
			}
		}
		else
		{
			for (int j = 0; j < bobjects.size();j++)
				bobjects.at(j)->visited = false;

			if (gModel->objects.size() > 0) 
			{
				int objs = 1;
				for(list<mapinect::ModelObject*>::iterator k = gModel->objects.begin(); 
					k != gModel->objects.end(); k++)
				{
					PCPolyhedron* hedron = (PCPolyhedron*)(*k);
					int hedronId = hedron->getId();
					BObject* curObj = getBObject(hedronId);
					curObj->clearSegments();
					curObj->update();
					vector<ofxVec3f> vecsproj;
					
					curObj->setPolyhedron(hedron);
					for (int i=0; i<hedron->getPCPolygonSize();i++)
					{
						ofxVec3f objCenter = hedron->getCenter();
						PCPolygon* gon = hedron->getPCPolygon(i);
						if (gon->hasObject()) 
						{
							mapinect::Polygon* q = gon->getPolygonModelObject();
							
							for(int i = 0; i < 4;i++)
							{
								ofxVec3f v = q->getVertex(i);

								ofxVec3f dif = v - tableCenter;
								ofxVec3f proj = dif.dot(tableNormal) * tableNormal;
								v = v - proj;
								vecsproj.push_back(v);
							}
						}			
					}	
					ofxVec3f centroid;
					vector<ofxVec3f> vecsprojunified = unifyVertex(vecsproj,centroid);
					if(vecsprojunified.size() > 2)
					{
						for(int i = 0; i < vecsprojunified.size() - 1; i++)
						{
							for(int j = i + 1; j < vecsprojunified.size(); j++)
							{
								Segment3D s1 = Segment3D(vecsprojunified.at(i),vecsprojunified.at(j),tableNormal,centroid,true);			
								curObj->pushSegment(s1);
							}
						}
					}
					else if (vecsprojunified.size() == 2)
					{
						Segment3D s1 = Segment3D(vecsprojunified.at(0),vecsprojunified.at(1),tableNormal,tableCenter,true,true);			
						curObj->pushSegment(s1);
					}
				}
			}
			ball.update(bobjects);
		}
		clearUnvisitedObjects();
		gModel->objectsMutex.unlock();
		
		
	}

	//--------------------------------------------------------------
	void BouncingBall::keyPressed(int key) {
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
			case '8':
				y_angle += 1.5;
				printf("yProj increased: %f \n",yProj);
				break;
			case '9':
				y_angle -= 1.5;
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
					ofBeginCustomFullscreen(1440, 0, 1280, 768);
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
		case 'r':
				//vA = ofxVec3f(-0.41767159,-0.098557055,0.5);//q->getVertex(0);
				//vB = ofxVec3f(0.24807897, 0.16238400, 0.5);//q->getVertex(1);
				//vC = ofxVec3f(0.28137761, -0.074146271, 0.5);//q->getVertex(2);
				//vD = ofxVec3f(-0.26263523,0.14728071, 0.5);//q->getVertex(3);
				//	
				//ofxVec3f w = ofxVec3f(0,0,1);//gon->getNormal();
				//ofxVec3f center = ofxVec3f(0, 0.1, 0.5);//hedron->getCenter();
				//float r = (rand()%100)/100.0;
				//ofxVec3f ballDir = ofxVec3f((rand()%100)/100.0,(rand()%100)/100.0,0); //w.cross(q->getVertex(rand()%4));
				//	
				//ball = Tejo(center,0.01,ballDir,0.01,w);
				//Segment3D s1 = Segment3D(vA,vC);
				//Segment3D s2 = Segment3D(vA,vD);
				//Segment3D s3 = Segment3D(vB,vD);
				//Segment3D s4 = Segment3D(vB,vC);
				//	
				//segments.push_back(s1);
				//segments.push_back(s2);
				//segments.push_back(s3);
				//segments.push_back(s4);
				if(gModel->table != NULL)
				{
					PCPolyhedron* hedron = (PCPolyhedron*)(gModel->table);
					PCPolygon* gon = hedron->getPCPolygon(0);
					if (gon->hasObject()) 
					{
						Polygon* q = gon->getPolygonModelObject();
					vA = q->getVertex(0);
					vB = q->getVertex(1);
					vC = q->getVertex(2);
					vD = q->getVertex(3);
					
					ofxVec3f center = hedron->getCenter();
					ofxVec3f w = ((vA - vC).cross(vA - vD)).normalize();//gon->getNormal();
					
					ofxVec3f ballDir = w;
					ballDir.cross(vC - vA);
					ballDir.rotate(rand()%360,w);
					ballDir *= -1;
						//-(w.cross(vC - vA)).rotate(10,w);
					
					ball = Tejo(center,0.01,ballDir,0.005,w,tableCenter);
					Segment3D s1 = Segment3D(vA,vB,w,center);
					Segment3D s2 = Segment3D(vB,vC,w,center);
					Segment3D s3 = Segment3D(vC,vD,w,center);
					Segment3D s4 = Segment3D(vD,vA,w,center);
					
					tableSegment3Ds.push_back(s1);
					tableSegment3Ds.push_back(s2);
					tableSegment3Ds.push_back(s3);
					tableSegment3Ds.push_back(s4);
					//segments.push_back(s6);

					tableSetted = true;
					}	
				}
			break;
		}
	}

	//--------------------------------------------------------------
	void BouncingBall::mouseMoved(int x, int y)
	{
	}

	//--------------------------------------------------------------
	void BouncingBall::mouseDragged(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void BouncingBall::mousePressed(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void BouncingBall::mouseReleased(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void BouncingBall::windowResized(int w, int h)
	{
	}

}
