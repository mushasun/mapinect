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
		if(!tableSetted)
		{
			ofxScopedMutex osm(gModel->tableMutex);
			if(gModel->getTable().get() != NULL)
			{
				mapinect::Polygon* t = gModel->getTable()->getPolygonModelObject();
				ofVec3f vA, vB, vC, vD;
				vA = t->getVertexs()[0];
				vB = t->getVertexs()[1];
				vC = t->getVertexs()[2];
				vD = t->getVertexs()[3];
					
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
				table->setModelObject(gModel->getTable().get());
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

			ofxScopedMutex osm(gModel->objectsMutex);
			if (gModel->getObjects().size() > 0) 
			{
				int objs = 1;
				for(vector<mapinect::ModelObjectPtr>::const_iterator k = gModel->getObjects().begin(); 
					k != gModel->getObjects().end(); k++)
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
								ofVec3f v = q->getVertexs()[i];

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
		
		
	}

	//--------------------------------------------------------------
	void BouncingBall::keyPressed(int key)
	{
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
