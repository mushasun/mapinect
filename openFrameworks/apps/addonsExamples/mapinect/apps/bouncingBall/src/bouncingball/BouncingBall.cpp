#include "BouncingBall.h"

#include "Constants.h"
#include "Globals.h"
#include "PCPolyhedron.h"

namespace bouncing {

	vector<ofVec3f> unifyVertex(vector<ofVec3f> lst, ofVec3f& centroid)
	{
		centroid = ofVec3f();
		vector<ofVec3f> retList;
		bool unified = false;
		for(int i = 0; i < lst.size(); i++)
		{
			ofVec3f pto = lst.at(i);
			for(int j = 0; j < retList.size(); j++)
			{
				if(pto.distance(retList.at(j)) < mapinect::MAX_UNIFYING_DISTANCE_PROJECTION)
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
		y_angle = 0.0;
		tableSetted = false;
	}

	//--------------------------------------------------------------
	void BouncingBall::exit()
	{

	}

	//--------------------------------------------------------------
	void BouncingBall::debugDraw()
	{

	}

	//--------------------------------------------------------------
	void BouncingBall::draw()
	{
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
	}

	vector<ofVec3f> BouncingBall::closerToTable(vector<ofVec3f> lst)
	{
		vector<ofVec3f> lstret;
		lstret.push_back(lst.at(0));
		for(int i = 1; i < lst.size(); i++)
		{
			ofVec3f curPto = lst.at(i);
			bool inserted = false;
			for (std::vector<ofVec3f>::const_iterator it = lstret.begin (); it != lstret.end (); ++it)
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

	
	void closestPoints(vector<ofVec3f> lst, ofVec3f center, ofVec3f& pt1,  ofVec3f& pt2)
	{
		float dist1 = MAX_FLOAT;
		float dist2 = MAX_FLOAT;

		for(int i = 0; i < lst.size(); i++)
		{
			ofVec3f pto = lst.at(i);
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
		BObject* nuObject = new BObject(nuVec,ofVec3f(rand()%256,rand()%256,rand()%256),id,rand()%6);
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
				mapinect::Polygon* t = gModel->table->getPolygonModelObject();
				ofVec3f vA, vB, vC, vD;
				vA = t->getVertex(0);
				vB = t->getVertex(1);
				vC = t->getVertex(2);
				vD = t->getVertex(3);
					
				ofVec3f center = t->getCenter();
				ofVec3f w = ((vA - vC).getCrossed(vA - vD)).normalize();//gon->getNormal();
				tableNormal = w;
				tableCenter = center;

				ofVec3f ballDir = w;
				ballDir.cross(vC - vA);
				ballDir.rotate(10,w);
				ballDir *= -1;
					//-(w.cross(vC - vA)).rotate(10,w);
					
				ball = Tejo(center,0.005,ballDir,0.0025,w,tableCenter);
				Segment3D s1(vA,vB,w,center);
				Segment3D s2(vB,vC,w,center);
				Segment3D s3(vC,vD,w,center);
				Segment3D s4(vD,vA,w,center);
					
				tableSegment3Ds.push_back(s1);
				tableSegment3Ds.push_back(s2);
				tableSegment3Ds.push_back(s3);
				tableSegment3Ds.push_back(s4);
				table = new BObject(tableSegment3Ds, ofVec3f(255,255,255), -1,0);
				table->setModelObject(gModel->table.get());
				//segments.push_back(s5);
				//segments.push_back(s6);
				bobjects.push_back(table);
				tableSetted = true;
			}
		}
		else
		{
			for (int j = 0; j < bobjects.size();j++)
				bobjects.at(j)->visited = false;

			if (gModel->objects.size() > 0) 
			{
				int objs = 1;
				for(vector<mapinect::ModelObjectPtr>::iterator k = gModel->objects.begin(); 
					k != gModel->objects.end(); k++)
				{
					PCPolyhedron* hedron = (PCPolyhedron*)(k->get());
					int hedronId = hedron->getId();
					BObject* curObj = getBObject(hedronId);
					curObj->clearSegments();
					curObj->update();
					vector<ofVec3f> vecsproj;
					
					curObj->setModelObject(hedron);
					for (int i=0; i<hedron->getPCPolygonSize();i++)
					{
						ofVec3f objCenter = hedron->getCenter();
						PCPolygonPtr gon = hedron->getPCPolygon(i);
						if (gon->hasObject()) 
						{
							mapinect::Polygon* q = gon->getPolygonModelObject();
							
							for(int i = 0; i < 4;i++)
							{
								ofVec3f v = q->getVertex(i);

								ofVec3f dif = v - tableCenter;
								ofVec3f proj = dif.dot(tableNormal) * tableNormal;
								v = v - proj;
								vecsproj.push_back(v);
							}
						}			
					}	
					ofVec3f centroid;
					vector<ofVec3f> vecsprojunified = unifyVertex(vecsproj,centroid);
					if(vecsprojunified.size() > 2)
					{
						for(int i = 0; i < vecsprojunified.size() - 1; i++)
						{
							for(int j = i + 1; j < vecsprojunified.size(); j++)
							{
								Segment3D s1 = Segment3D(vecsprojunified.at(i),vecsprojunified.at(j),tableNormal,centroid,true);			
								curObj->addSegment(s1);
							}
						}
					}
					else if (vecsprojunified.size() == 2)
					{
						Segment3D s1 = Segment3D(vecsprojunified.at(0),vecsprojunified.at(1),tableNormal,tableCenter,true,true);			
						curObj->addSegment(s1);
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
		switch (key) {
		case 'r':
			//vA = ofVec3f(-0.41767159,-0.098557055,0.5);//q->getVertex(0);
			//vB = ofVec3f(0.24807897, 0.16238400, 0.5);//q->getVertex(1);
			//vC = ofVec3f(0.28137761, -0.074146271, 0.5);//q->getVertex(2);
			//vD = ofVec3f(-0.26263523,0.14728071, 0.5);//q->getVertex(3);
			//	
			//ofVec3f w = ofVec3f(0,0,1);//gon->getNormal();
			//ofVec3f center = ofVec3f(0, 0.1, 0.5);//hedron->getCenter();
			//float r = (rand()%100)/100.0;
			//ofVec3f ballDir = ofVec3f((rand()%100)/100.0,(rand()%100)/100.0,0); //w.cross(q->getVertex(rand()%4));
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
				if (gModel->table->hasObject()) 
				{
					mapinect::Polygon* q = gModel->table->getPolygonModelObject();
					ofVec3f vA, vB, vC, vD;
					vA = q->getVertex(0);
					vB = q->getVertex(1);
					vC = q->getVertex(2);
					vD = q->getVertex(3);
					
					ofVec3f center = gModel->table->getCenter();
					ofVec3f w = ((vA - vC).cross(vA - vD)).normalize();//gon->getNormal();
					
					ofVec3f ballDir = w;
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
