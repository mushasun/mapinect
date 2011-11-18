#include "Buildings.h"

#include "utils.h"

namespace buildings {

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
		Floor::floorTexture = txManager->loadTexture("data/texturas/135367.jpg");
		Building::buildingTexture = txManager->loadTexture("data/texturas/Building_texture.jpg");
		Building::roofTexture = txManager->loadTexture("data/texturas/oba.jpg");
	}

	//--------------------------------------------------------------
	void Buildings::exit() {

	}

	//--------------------------------------------------------------
	void Buildings::draw()
	{
		ofxScopedMutex objectsLock(gModel->objectsMutex);

		if (floor == NULL && gModel->table != NULL) {
			PCPolyhedron* table = dynamic_cast<PCPolyhedron*>(gModel->table);
			if (table->getPCPolygonSize() > 0) {
				floor = new Floor(table->getPCPolygon(0));
			}
		}

		if (floor != NULL) {
			floor->draw(txManager);
		}

		for (list<mapinect::ModelObject*>::iterator iter = gModel->objects.begin(); iter != gModel->objects.end(); iter++) {
			int id = (*iter)->getId();
			if (buildings.find(id) == buildings.end()) {
				buildings[id] = new Building(id, dynamic_cast<PCPolyhedron*>(*iter));
			}
		}

		for (map<int, Building*>::iterator iter = buildings.begin(); iter != buildings.end(); iter++) {
			(iter->second)->draw(txManager, floor);
		}

		/*
		if ( > 1)
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
		*/
	}


	//--------------------------------------------------------------
	void Buildings::update() {
	
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
