#include "Buildings.h"

#include "Globals.h"
#include "pointUtils.h"		// Esto vamos a tener que sacarlo

namespace buildings {
	
	GLuint Buildings::videoTexture = 0;
	GLuint Buildings::videoTexture2 = 0;

	struct Camera 
	{
	public:
		Camera(const ofVec3f& position, const ofVec3f& lookAt)
			: position(position), lookAt(lookAt) { }
		ofVec3f position;
		ofVec3f lookAt;
	};

	vector<Camera> CAMERAS;

	//--------------------------------------------------------------
	Buildings::Buildings() {
		floor = NULL;
	}

	//--------------------------------------------------------------
	Buildings::~Buildings() {
		if (floor != NULL) {
			delete floor;
		}
		for (map<int, Building*>::iterator i = buildings.begin(); i != buildings.end(); i++) {
			delete i->second;
		}
		buildings.clear();
	}

	//--------------------------------------------------------------
	void Buildings::setup() {
		/*
		videoTexture = txManager->loadVideoTexture("data/movies/fingers.mov"); // En: mapinect\apps\buildings\bin\data\movies
		videoTexture2 = txManager->loadVideoTexture("data/movies/MOV05377.MPG");  
		*/

		Camera camera1(ofVec3f(0.37,0.0,0.0), ofVec3f(2.0,0.30,0.35));
		CAMERAS.push_back(camera1);

		Camera camera2(ofVec3f(0.37,0.0,0.0), ofVec3f(2.0,0.30,-0.25));
		CAMERAS.push_back(camera2);

		Camera camera3(ofVec3f(0.25,0.0,0.25), ofVec3f(2.0,0.30,-1.00));
		CAMERAS.push_back(camera3);

		Camera camera4(ofVec3f(0.0,0.0,-0.35), ofVec3f(2.0,0.30,-0.1));
		CAMERAS.push_back(camera4);

	}

	//--------------------------------------------------------------
	void Buildings::exit() {
	}

	//--------------------------------------------------------------
	void Buildings::debugDraw()
	{

	}

	//--------------------------------------------------------------
	void Buildings::draw()
	{
		if (floor != NULL) {
			floor->draw();
		}

		for (map<int, Building*>::iterator iter = buildings.begin(); iter != buildings.end(); iter++) {
			(iter->second)->draw(*floor);
		}

/*		
		if ( > 1)
		{
			//termine de procesar los objetos, tengo los centroides en centroides
			//a dibujar la calle
			
			//obtengo datos de la mesa
			PCPolyhedron* mesa = (PCPolyhedron*)(gModel->table);
			ofVec3f tableCenter = mesa->getCenter();
			ofVec3f tableNormal = mesa->getPCPolygon(0)->getNormal();

			//obtengo la proyeccion del primer punto
			ofVec3f primero = centroides.back();
			ofVec3f dif = primero - tableCenter;
			ofVec3f proj = dif.dot(tableNormal) * tableNormal;
			primero = primero - proj;
			centroides.pop_back();

			//paso al segundo
			ofVec3f segundo = centroides.back();
			dif = segundo - tableCenter;
			proj = dif.dot(tableNormal) * tableNormal;
			segundo = segundo - proj;
			centroides.pop_back();


			ofVec3f distancia = primero - segundo;
			ofVec3f vector_ancho = distancia.cross(tableNormal)/10;

			static ofVec3f vA,vB,vC,vD;

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

			glEnable(GL_TEXTURE_2D);
			glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
			// GL_REPLACE can be specified to just draw the surface using the texture colors only
			// GL_MODULATE means the computed surface color is multiplied by the texture color (to be used when lighting)
			glBindTexture(GL_TEXTURE_2D, camino); 
*/
			// Draw quad and map 2D points for texture mapping
	/*		
			glDisable(GL_TEXTURE_2D); // Draw colored side faces
			glBegin(GL_TRIANGLE_STRIP);
				glColor3f(0.8f,0.8f,0.8f);
				glVertex3f(0.0f, 0.0f, 1.0f);
				glVertex3f(0.0f, 0.1f, 1.0f);
				glVertex3f(0.1f, 0.0f, 1.0f);
			glEnd();
			*/
//			glEnable(GL_TEXTURE_2D); // Draw colored side faces
		
/*			// Bind, update and draw texture 1
			txManager->bindTexture(videoTexture);
			//txManager->updateVideoTexture(videoTexture);
			glBegin(GL_QUADS);
				glTexCoord2f(0, 0);
				glVertex3f(0.0,0.0,3.0);    
				//	glVertex3f(vA.x, vA.y, vA.z);
				glTexCoord2f(1, 0);
				glVertex3f(1.0,0.0,3.0);
				//	glVertex3f(vB.x, vB.y, vB.z);
				glTexCoord2f(1, 1);
				glVertex3f(1.0,1.0,3.0);
				//	glVertex3f(vC.x, vC.y, vC.z);
				glTexCoord2f(0, 1);
				glVertex3f(0.0,1.0,3.0);    
				//	glVertex3f(vD.x, vD.y, vD.z);
			glEnd();

			// Bind, update and draw texture 2
			txManager->bindTexture(videoTexture2);
			//txManager->updateVideoTexture(videoTexture2);
			glBegin(GL_QUADS);
				glTexCoord2f(0, 0);
				glVertex3f(-1.0,0.0,2.0);    
				//	glVertex3f(vA.x, vA.y, vA.z);
				glTexCoord2f(1, 0);
				glVertex3f(0.0,0.0,2.0);
				//	glVertex3f(vB.x, vB.y, vB.z);
				glTexCoord2f(1, 1);
				glVertex3f(0.0,1.0,2.0);
				//	glVertex3f(vC.x, vC.y, vC.z);
				glTexCoord2f(0, 1);
				glVertex3f(-1.0,1.0,2.0);    
				//	glVertex3f(vD.x, vD.y, vD.z);
			glEnd();
*/

	/*	}
		*/
	}


	//--------------------------------------------------------------
	void Buildings::update() {
	//	txManager->updateVideoTexture();
	}

	//--------------------------------------------------------------
	void Buildings::keyPressed(int key)
	{
		switch (key) {
			case '1':
				setCamera(1);
				break;
			case '2':
				setCamera(2);
				break;
			case '3':
				setCamera(3);
				break;
			case '4':
				setCamera(4);
				break;
		}
	}

	void Buildings::setCamera(int camera)
	{
		Camera cam = CAMERAS.at(camera - 1);

		this->armController->setArmPositionAndLookAt(cam.position,cam.lookAt);	
	}


	//--------------------------------------------------------------
	void Buildings::keyReleased(int key)
	{
	}

	//--------------------------------------------------------------
	void Buildings::windowMoved(int x, int y)
	{
	}

	//--------------------------------------------------------------
	void Buildings::mouseMoved(int x, int y)
	{
	}

	//--------------------------------------------------------------
	void Buildings::mouseDragged(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void Buildings::mousePressed(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void Buildings::mouseReleased(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void Buildings::dragEvent(ofDragInfo info)
	{
	}

	//--------------------------------------------------------------
	void Buildings::objectDetected(const IObjectPtr& object)
	{
		if (object->getId() == TABLE_ID)
		{
			if (floor != NULL)
			{
				delete floor;
				floor = NULL;
			}
			if (floor == NULL)
			{
				floor = new Floor(object->getPolygons()[0]);
			}
		}
		else
		{
			if (buildings.find(object->getId()) == buildings.end())
			{
				buildings[object->getId()] = new Building(object);
				cout << "Se detecto un nuevo objeto, con id=" << object->getId() << endl;
				if (object->getId() == 1) 
				{
					// Seguir solo al objeto con id = 1
					armController->followObject(object); // ->lookAtObject(object);
				}

			} else {
				cout << "El objeto con id=" << object->getId() << " ya existe" << endl;
			}
		}
	}

	//--------------------------------------------------------------
	void Buildings::objectUpdated(const IObjectPtr& object)
	{
		if (object->getId() == TABLE_ID)
		{
			if (floor != NULL)
			{
				floor->updateModelObject(object->getPolygons()[0]);
			}
		}
		else
		{
			map<int, Building*>::iterator b = buildings.find(object->getId());
			if (b != buildings.end())
			{
				b->second->updateModelObject(object);
			}
		}
	}

	//--------------------------------------------------------------
	void Buildings::objectLost(const IObjectPtr& object)
	{
		if (object->getId() == TABLE_ID)
		{
			if (floor != NULL)
			{
				delete floor;
				floor = NULL;
			}
		}
		else
		{
			if (buildings.find(object->getId()) != buildings.end())
			{
				buildings.erase(object->getId());
			}
		}
	}

	//--------------------------------------------------------------
	void Buildings::objectMoved(const IObjectPtr& object, const DataMovement& movement)
	{
		if (object->getId() == TABLE_ID)
		{
			if (floor != NULL)
			{
				floor->updateModelObject(object->getPolygons()[0]);
			}
		}
		else
		{
			map<int, Building*>::iterator b = buildings.find(object->getId());
			if (b != buildings.end())
			{
				b->second->updateModelObject(object);
			}
		}
	}
	
	//--------------------------------------------------------------
	void Buildings::objectTouched(const IObjectPtr& object, const DataTouch& touchPoint)
	{
		map<int, DataTouch>::iterator it = touchPoints.find(touchPoint.getId());
		if (it == touchPoints.end())
		{
			assert(touchPoint.getType() == kTouchTypeStarted);
			touchPoints.insert(make_pair(touchPoint.getId(), touchPoint));
		}
		else
		{
			it->second = touchPoint;
		}
	}
}
