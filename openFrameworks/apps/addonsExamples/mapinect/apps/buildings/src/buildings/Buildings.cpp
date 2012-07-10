#include "Buildings.h"

#include "Globals.h"
#include "pointUtils.h"		// Esto vamos a tener que sacarlo

namespace buildings {
	
	GLuint Buildings::videoTexture = 0;
	GLuint Buildings::videoTexture2 = 0;

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
	}

	//--------------------------------------------------------------
	void Buildings::exit() {
	}

	//--------------------------------------------------------------
	void Buildings::debugDraw()
	{
		// Esto vamos a tener que sacarlo
		map<int, DataTouch> keep;
		for (map<int, DataTouch>::const_iterator it = touchPoints.begin(); it != touchPoints.end(); ++it)
		{
			if (it->second.getType() == kTouchTypeStarted)
				ofSetHexColor(0xFF0000);
			else if (it->second.getType() == kTouchTypeHolding)
				ofSetHexColor(0x00FF00);
			else
				ofSetHexColor(0x0000FF);
			ofVec2f s = getScreenCoords(it->second.getTouchPoint());
			ofCircle(s.x, s.y, 4);
			if (it->second.getType() != kTouchTypeReleased)
				keep.insert(make_pair(it->first, it->second));
		}
		touchPoints = keep;
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
			}
		}
		armController->lookAtObject(object);
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
