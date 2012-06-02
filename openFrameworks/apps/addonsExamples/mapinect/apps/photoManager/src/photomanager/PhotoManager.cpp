#include "PhotoManager.h"

#include "Globals.h"

namespace photo {
float weight[32] = {0.0294,0.0331,0.0654,0.0756,0.0554,0.0314,0.0454,0.0469,0.0956,0.0763,0.1100,0.0676,0.0755,0.0500,0.0667,0.0749,
						0.0637,0.0516,0.0864,0.0636,0.0747,0.0365,0.0349,0.0649,0.0656,0.1189,0.0362,0.0849,0.0368,0.0389,0.0943,0.0477};
	ofVec3f covariance[32] = {
		ofVec3f	(765.40, 121.44, 112.80) ,
		ofVec3f	(39.94, 154.44, 396.05) 	,
		ofVec3f	(291.03, 60.48, 162.85) 	,
		ofVec3f	(274.95, 64.60, 198.27) 	,
		ofVec3f	(633.18, 222.40, 250.69) 	,
		ofVec3f	(65.23, 691.53, 609.92) 	,
		ofVec3f	(408.63, 200.77, 257.57) 	,
		ofVec3f	(530.08, 155.08, 572.79) 	,
		ofVec3f	(160.57, 84.52, 243.90) 	,
		ofVec3f	(163.80, 121.57, 279.22) 	,
		ofVec3f	(425.40, 73.56, 175.11) 	,
		ofVec3f	(330.45, 70.34, 151.82) 	,
		ofVec3f	(152.76, 92.14, 259.15) 	,
		ofVec3f	(204.90, 140.17, 270.19) 	,
		ofVec3f	(448.13, 90.18, 151.29) 	,
		ofVec3f	(178.38, 156.27, 404.99) 	,

		//Not Hand
		ofVec3f	 (2.77, 2.81, 5.46)	,
		ofVec3f	 (46.84, 33.59, 32.48)	,
		ofVec3f	 (280.69, 156.79, 436.58)	,
		ofVec3f	 (355.98, 115.89, 591.24)	,
		ofVec3f	 (414.84, 245.95, 361.27) 	,
		ofVec3f	 (2502.24, 1383.53, 237.18)	,
		ofVec3f	 (957.42, 1766.94, 1582.52)	,
		ofVec3f	 (562.88, 190.23, 447.28)	,
		ofVec3f	 (344.11, 191.77, 433.40)	,
		ofVec3f	 (222.07, 118.65, 182.41)	,
		ofVec3f	(651.32, 840.52, 963.67)	,
		ofVec3f	 (225.03, 117.29, 331.95)	,
		ofVec3f	 (494.04, 237.69, 533.52)	,
		ofVec3f	 (955.88, 654.95, 916.70)	,
		ofVec3f	 (350.35, 130.30, 388.43)	,
		ofVec3f	(806.44, 642.20, 350.36)	

	};
	ofVec3f mean[32] = {
		ofVec3f	(73.53, 29.94, 17.76)	,
		ofVec3f	(249.71, 233.94, 217.49)	,
		ofVec3f	(161.68, 116.25, 96.95)	,
		ofVec3f	(186.07, 136.62, 114.40) 	,
		ofVec3f	(189.26, 98.37, 51.18) 	,
		ofVec3f	(247.00, 152.20, 90.84) 	,
		ofVec3f	(150.10, 72.66, 37.76) 	,
		ofVec3f	(206.85, 171.09, 156.34) 	,
		ofVec3f	(212.78, 152.82, 120.04) 	,
		ofVec3f	(234.87, 175.43, 138.94) 	,
		ofVec3f	(151.19, 97.74, 74.59) 	,
		ofVec3f	(120.52, 77.55, 59.82) 	,
		ofVec3f	(192.20, 119.62, 82.32) 	,
		ofVec3f	(214.29, 136.08, 87.24) 	,
		ofVec3f	(99.57, 54.33, 38.06) 	,
		ofVec3f	(238.88, 203.08, 176.91) 	,


		//Not hand
		ofVec3f	(254.37, 254.41, 253.82)	,
		ofVec3f	(9.39, 8.09, 8.52)	,
		ofVec3f	(96.57, 96.95, 91.53)	,
		ofVec3f	(160.44, 162.49, 159.06)	,
		ofVec3f	(74.98, 63.23, 46.33)	,
		ofVec3f	(121.83, 60.88, 18.31)	,
		ofVec3f	(202.18, 154.88, 91.04)	,
		ofVec3f	(193.06, 201.93, 206.55)	,
		ofVec3f	(51.88, 57.14, 61.55)	,
		ofVec3f	(30.88, 26.84, 25.32)	,
		ofVec3f	(44.97, 85.96, 131.95) 	,
		ofVec3f	(236.02, 236.27, 230.70)	,
		ofVec3f	(207.86, 191.20, 164.12)	,
		ofVec3f	(99.83, 148.11, 188.17)	,
		ofVec3f	(135.06, 131.92, 123.10)	,
		ofVec3f	(135.96, 103.89, 66.88) 	


	};

	float handProbability(ofVec3f pixel, bool negate)
	{
		int offset = 0;
		float pi_pow = pow(2*PI,1.5);
		if(negate)
			offset += 16;
		float result = 0;
		for(int i = offset; i < 16 + offset; i++)
		{
			float covarianceDet = covariance[i].x * covariance[i].y * covariance[i].z;
			float term1 = weight[i]*(1/(pi_pow*pow(covarianceDet,0.5f)));
			
			ofVec3f pixelMean = pixel - mean[i];
			ofVec3f cov = covariance[i];
			float e_elevated = ((pixelMean.x * pixelMean.x)/cov.x) + 
							   ((pixelMean.y * pixelMean.y)/cov.y) + 
							   ((pixelMean.z * pixelMean.z)/cov.z);
			e_elevated *= -0.5;

			result += term1*exp(e_elevated);

		}
		return result;
	}

	list<ofPoint> skinPoints;

	void PhotoManager::keyPressed(int key) {
		int counter = 0;
		skinPoints.clear();
		for(int i = 0; i < 640; i++)
		{
			for(int j = 0; j < 480; j++)
			{
				ofColor c = gKinect->getColorAt(i,j);
				ofVec3f c_v = ofVec3f(c.r,c.g,c.b);
				//c_v /= 255;
				float prob = handProbability(c_v,false);// / handProbability(c_v,true);
//				cout << prob << endl;
				if(prob > 0.6)
				{
					skinPoints.push_back(ofPoint(i,j));
					counter ++;
					//cout << "MANO!" << endl;
				}
			}
		}
		cout << counter << endl;
	}

	GLuint PhotoManager::tableTexture = 0;

	//--------------------------------------------------------------
	void PhotoManager::setup() {
		/*PhotoManager::tableTexture = txManager->loadTexture("data/texturas/brickPath_2.jpg");
		Photo* photo = new Photo();
		photo->texture = txManager->loadTexture("data/texturas/DSC02568.JPG");
		table = NULL;
		photos.push_back(photo);*/
	}

	void PhotoManager::exit()
	{

	}

	//--------------------------------------------------------------
	#define KWIDTH		640
	#define KHEIGHT		480
	void PhotoManager::debugDraw()
	{
		/*unsigned char* pixels = gKinect->getCalibratedRGBPixels();
		
		for (list<ofPoint>::iterator iter = skinPoints.begin(); iter != skinPoints.end(); iter++) {
			int index = (iter->y * KWIDTH + iter->x) * 3;
			pixels[index] = 255;
			pixels[index+1] = 0;
			pixels[index+2] = 0;
		}*/

		/*for (int j = 0; j < KHEIGHT; j++) {
			for (int i = 0; i < KWIDTH; i++) {
				int ix = (j * KWIDTH + i) * 3;
				pixels[ix] = 255.0f;
				pixels[ix + 1] = 0.0f;
				pixels[ix + 2] = 0.0f;
			}
		}*/


		/*ofImage img;
		glEnable (GL_BLEND);
		glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		img.setFromPixels(pixels,640,480,OF_IMAGE_COLOR);
		img.draw(0,0);
*/
		
		
		//for(int i = 0; i < 640; i++)
		//{
		//	for(int j = 0; j < 480; j++)
		//	{
		//		ofColor c = gKinect->getColorAt(i,j);
		//		glBegin(GL_POINTS);
		//		ofSetColor(c.r,c.g,c.b);
		//		ofVertex(i,j);
		//		glEnd();
		//	}
		//
		//}
	}
	void PhotoManager::draw()
	{
		//Dibujo Mesa
	//	if (table != NULL) 
	//	{
	//		mapinect::Polygon* q = table->getPolygonModelObject();

	//		//txManager->enableTextures();
	//		//txManager->bindTexture(PhotoManager::tableTexture);

	//		//ofDrawQuadTextured(q->getVertex(0), q->getVertex(1), q->getVertex(2), q->getVertex(3));

	//		//txManager->disableTextures();

	///*		ofSetColor(255,0,0);
	//		glBegin(GL_QUADS);
	//		glVertex3f(q->getVertex(0).x, q->getVertex(0).y, q->getVertex(0).z);
	//		glVertex3f(q->getVertex(1).x, q->getVertex(1).y, q->getVertex(1).z);
	//		glVertex3f(q->getVertex(2).x, q->getVertex(2).y, q->getVertex(2).z);
	//		glVertex3f(q->getVertex(3).x, q->getVertex(3).y, q->getVertex(3).z);
	//		glEnd();*/
	//	}

	//	for(list<Photo*>::iterator photoIter = photos.begin(); photoIter != photos.end(); photoIter++){
	//		(*photoIter)->draw(txManager);
	//	}
	//	for (list<Grab>::iterator grabIter = grabs.begin(); grabIter != grabs.end(); grabIter++) {
	//		grabIter->draw();
	//	}

	
	}

	
	//--------------------------------------------------------------
	void PhotoManager::update() {
		//if(table == NULL && gModel->table != NULL)
		//{
		//	PCPolyhedron* tablePol = dynamic_cast<PCPolyhedron*>(gModel->table);
		//	if (tablePol->getPCPolygonSize() > 0) {
		//		table = tablePol->getPCPolygon(0);
		//	}

		//	ofVec3f center = tablePol->getCenter();
		//	//Inicializo las fotos
		//	for(list<Photo*>::iterator photoIter = photos.begin(); photoIter != photos.end(); photoIter++){
		//		(*photoIter)->setNormal(table->getNormal());
		//		(*photoIter)->setPos(center);
		//		(*photoIter)->width = 0.1;
		//		(*photoIter)->height = 0.2;
		//	}
		//}
		//else
		//{
		//	list<PCHand*> pendingHands;
		//	for (list<ModelObject*>::iterator handIter = gModel->objects.begin(); handIter != gModel->objects.end(); handIter++) {
		//		PCHand* hand = dynamic_cast<PCHand*>(*handIter);
		//		bool grabUpdated = false;
		//		for (list<Grab>::iterator grabIter = grabs.begin(); grabIter != grabs.end() && !grabUpdated; grabIter++) {
		//			grabUpdated = grabIter->Update(hand);
		//		}
		//		if(!grabUpdated)
		//			pendingHands.push_back(hand);
		//	}

		//	for (list<PCHand*>::iterator handIter = pendingHands.begin(); handIter != pendingHands.end(); handIter++) {
		//		bool handGrabPhoto = false;
		//		for(list<Photo*>::iterator photoIter = photos.begin(); photoIter != photos.end() && !handGrabPhoto; photoIter++){
		//			handGrabPhoto = isHandInPhoto(*handIter,*photoIter);
		//		}
		//	}
		//}
	}
	
	//--------------------------------------------------------------
	bool PhotoManager::isHandInPhoto(PCHand* hand,Photo* photo)
	{
		vector<ofVec3f> fingerTips = hand->getFingerTips();
		bool handInPhoto = false;
		for(vector<ofVec3f>::iterator iter = fingerTips.begin(); iter != fingerTips.end() && !handInPhoto; iter++)
		{
			ofVec3f tip = *iter;

			ofVec3f pos = photo->getPos();
			
			//TODO: Corregir esta condicion!
			handInPhoto = tip.x > pos.x && tip.x < pos.x + photo->width &&
						  tip.y > pos.y && tip.y < pos.y + photo->height;
		}

		if(handInPhoto)
		{
			Grab nuGrab = Grab(photo, hand);
			grabs.push_back(nuGrab);
		}

		return handInPhoto;
	}

	//--------------------------------------------------------------


	//--------------------------------------------------------------
	void PhotoManager::mouseMoved(int x, int y)
	{
	}

	//--------------------------------------------------------------
	void PhotoManager::mouseDragged(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void PhotoManager::mousePressed(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void PhotoManager::mouseReleased(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void PhotoManager::windowResized(int w, int h)
	{
	}

}
