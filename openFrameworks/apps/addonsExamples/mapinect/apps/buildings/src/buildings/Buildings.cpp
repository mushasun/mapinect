#include "Buildings.h"

#include "Globals.h"
#include "utils.h"

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
		Floor::floorTexture = txManager->loadImageTexture("data/texturas/135367.jpg");
		Building::buildingTexture = txManager->loadImageTexture("data/texturas/Building_texture.jpg");
		Building::roofTexture = txManager->loadImageTexture("data/texturas/oba.jpg");
		/*
		videoTexture = txManager->loadVideoTexture("data/movies/fingers.mov"); // En: mapinect\apps\buildings\bin\data\movies
		videoTexture2 = txManager->loadVideoTexture("data/movies/MOV05377.MPG");  
		*/
	}

	//--------------------------------------------------------------
	void Buildings::exit() {
		txManager->unloadTexture(videoTexture);
		txManager->unloadTexture(videoTexture2);
	}

	//--------------------------------------------------------------
	void Buildings::debugDraw()
	{

	}

	//--------------------------------------------------------------
	void Buildings::draw()
	{
//		ofxScopedMutex objectsLock(gModel->objectsMutex);

		gModel->objectsMutex.lock();

		if (floor == NULL && gModel->table != NULL) {
			floor = new Floor(gModel->table);
		}

		if (floor != NULL) {
			floor->draw(txManager);
		}


		for (vector<mapinect::ModelObjectPtr>::iterator iter = gModel->objects.begin(); iter != gModel->objects.end(); iter++) {
			int id = (*iter)->getId();
			if (buildings.find(id) == buildings.end()) {
				buildings[id] = new Building(id, PCPolyhedronPtr(dynamic_cast<PCPolyhedron*>(iter->get())));
			}
		}

		for (map<int, Building*>::iterator iter = buildings.begin(); iter != buildings.end(); iter++) {
			(iter->second)->draw(txManager, *floor);
		}

		gModel->objectsMutex.unlock();

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
	void Buildings::windowResized(int w, int h)
	{
	}

}
