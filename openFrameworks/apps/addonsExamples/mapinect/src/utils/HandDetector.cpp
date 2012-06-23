#include "HandDetector.h"

#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/convex_hull.h>

#include "ofGraphics.h"

#include "Constants.h"
#include "pointUtils.h"
#include "Table.h"
#include "utils.h"

namespace mapinect {

	void HandDetector::SetPotentialHandCloud(const PCPtr& cloud)
	{
		this->hand = cloud;
	}

	void HandDetector::SetTable(const TablePtr& table)
	{
		this->table = table;
	}

	//Elimino puntos por fuera de la mesa
	//Busco el extremo de la mano
	//Determino la dirección de la mano
	void HandDetector::trimPointsOutsideTable()
	{
		float min_x, min_y, min_z, max_x, max_y, max_z;
		float minDif_minX = numeric_limits<float>::max();
		float minDif_maxX = numeric_limits<float>::max();
		int idx_minDif_minX = -1; 
		int idx_minDif_maxX = -1;

		ofVec3f min2, max2;
		PCPtr tableCloud (table->getCloud());
		computeBoundingBox(tableCloud, min2, max2);
		//createCloud(min2, "min2.pcd");
		//createCloud(max2, "max2.pcd");
	

		//Calculo el bounding box
		min_x = min(min2.x,max2.x);
		min_y = min(min2.y,max2.y);
		min_z = min(min2.z,max2.z);
		max_x = max(min2.x,max2.x);
		max_y = max(min2.y,max2.y);
		max_z = max(min2.z,max2.z);

		//pcl::io::savePCDFileASCII ("hand_preCliped.pcd", *hand); 

		PCPtr filteredcloud (new PC());
		for(int i = 0; i < hand->size(); i ++)
		{
			pcl::PointXYZ pto = hand->at(i);
			if(pto.x < max_x && pto.x > min_x &&	// Chequeo que esté dentro del BB
			   pto.z < max_z && pto.z > min_z)		//
			{
				filteredcloud->push_back(pto);
				if(abs(pto.x - min_x) < minDif_minX){	//
					minDif_minX = abs(pto.x - min_x);	//	Busco los puntos que estén mas cerca del borde
					idx_minDif_minX = i;				//
				}										//
				if(abs(pto.x - max_x) < minDif_maxX){	//
					minDif_maxX = abs(pto.x - max_x);	//
					idx_minDif_maxX = i;				//
				}
			}
		}
	
		if(idx_minDif_minX != -1 && idx_minDif_maxX != -1)
		{
			if(minDif_minX < minDif_maxX)
			{
				tipPoint = hand->at(idx_minDif_maxX);
				handDirection = -1; //Mano desde izq de kinect
			}
			else
			{
				tipPoint = hand->at(idx_minDif_minX);
				handDirection = 1;
			}
		}

		hand = filteredcloud;
		saveCloud("hand_Cliped.pcd", *filteredcloud);
	}

	//Corta la mano desde la punta hasta X cm
	void HandDetector::trimHand()
	{
			float maxXFilter = tipPoint.x;
			float minXFilter = tipPoint.x + (mapinect::HAND_SIZE * handDirection);

			pcl::PassThrough<pcl::PointXYZ> pass;
			pass.setInputCloud (hand);
			pass.setFilterFieldName ("x");
			if(handDirection == 1)
				pass.setFilterLimits (maxXFilter, minXFilter);
			else
				pass.setFilterLimits (minXFilter, maxXFilter);
			pass.filter (*hand);

			//pcl::io::savePCDFileASCII ("hand.pcd", *hand);
	}

	vector<ofVec3f> HandDetector::unifyHandVertex(const PCPtr& handHull)
		{
			//cout << "pre unify: " << handHull->size() << endl;
			vector<vector<ofVec3f>> tmp;
			vector<ofVec3f> final;
			//pcl::io::savePCDFileASCII ("preunify.pcd", *handHull);

			for (int i = 0; i < handHull->size(); i++) {
				ofVec3f pto = PCXYZ_OFVEC3F(handHull->at(i));
				bool unified = false;
				for(int j = 0; j < tmp.size(); j++)
				{
					vector<ofVec3f> inTmp = tmp.at(j);
					if(inTmp.size() > 0 && 
						inTmp.at(0).distance(pto) < mapinect::MAX_UNIFYING_DISTANCE_PROJECTION)
					{
						inTmp.push_back(pto);
						unified = true;
					}
				}
				if(!unified)
				{
					vector<ofVec3f> inTmp;
					inTmp.push_back(pto);
					tmp.push_back(inTmp);
				}
			}

			for(int i = 0; i < tmp.size(); i++)
			{
				vector<ofVec3f> inTmp = tmp.at(i);
				ofVec3f avg = ofVec3f();
				int j;
				for(j = 0; j < inTmp.size(); j++)
					avg += inTmp.at(j);
				avg /= j;
				final.push_back(avg);
			}

			//cout << "post unify: " << final.size() << endl;
			return final;
		}

	//Busco los dedos y seteo el centroide
	vector<ofVec3f> HandDetector::getFingers()
	{
		//aplano mano - debug
		for(int i = 0; i < hand->size(); i ++)
			hand->at(i).z = 0;

		pcl::io::savePCDFileASCII ("handflat.pcd", *hand);

		//Calculo convex hull
		PCPtr hand_hull (new PC());
		pcl::ConvexHull<pcl::PointXYZ> chull;
		chull.setInputCloud (hand);
		chull.setDimension(2);
		chull.reconstruct (*hand_hull);
		//writer.write<pcl::PointXYZ> ("handhull.pcd", *hand_hull, false);

		//Calculo el centroide
		Eigen::Vector4f vHandCentroid;
		pcl::compute3DCentroid(*hand,vHandCentroid);
		handCentroid = pcl::PointXYZ(vHandCentroid.x(),vHandCentroid.y(),vHandCentroid.z());
		PCPtr cloud_centroid (new PC());
		cloud_centroid->push_back(handCentroid);
		//pcl::io::savePCDFileASCII ("handCentroid.pcd", *cloud_centroid);

		//writer.write<pcl::PointXYZ> ("handCloud.pcd", *hand, false);
		//Unifico los vertices del hull encontrado
		return unifyHandVertex(hand_hull);
	}

	//Busco el dedo más cercano al dedo pasado por parámetro
	int HandDetector::findCloserFingerTo(const ofVec3f& currentFinger, const vector<ofVec3f>& unifiedHull, int handDirection)
	{
		int idx = -1;
		float minDist = numeric_limits<float>::max( );
		for(int i = 0; i < unifiedHull.size(); i ++)
		{
			if((currentFinger - unifiedHull.at(i)).length() < minDist)
			{
				minDist = (currentFinger - unifiedHull.at(i)).length();
				idx = i;
			}
		}

		return idx;
	}

	//Chequeo la cantidad de dedos y el ángulo
	float HandDetector::checkFingers(vector<ofVec3f>& fingers)
	{
		bool saveToFile = false;
		//cout << "check finger" << endl;
		//cout << "hand fingers: " << fingers.size() << endl;

		
		//probabilidad de que sea mano. Cada propiedad suma a la probabilidad:
		//largo correcto de cada dedo:		+.111
		//angulo correcto entre cada dedo:	+.111
		float prob = 0;

		//Busco el pulgar
		//El que tenga el x mas cercano al centroide y el 'y' mayor que el del centroide
		float minDist = 9999; //TODO: cambiar por max
		int minDistIdx = -1;
		for(int i = 0; i < fingers.size(); i++)
		{
			if(fingers.at(i).y < handCentroid.y)
			{
				if(abs(fingers.at(i).x - handCentroid.x) < minDist)
				{
					minDist = abs(fingers.at(i).x - handCentroid.x);
					minDistIdx = i;
				}
			}
		}

		if(minDistIdx == -1) //No encontre posible pulgar
			return false;

		ofVec3f vCentroid = ofVec3f(handCentroid.x,handCentroid.y, handCentroid.z);
		ofVec3f thumbTip = fingers.at(minDistIdx);
		saveCloud("thumb.pcd", thumbTip);

		//cout << "thumb x " << thumbTip.x << endl ;
		//Quito el pulgar del hull
		fingers.erase(fingers.begin() + minDistIdx);

		//Quito los puntos que estén 'por detras' del pulgar
		vector<ofVec3f> tmp;
		for(int i = 0; i < fingers.size(); i ++)
		{
			//cout << unifiedHull.at(i).x << endl;
			if((handDirection == -1 && fingers.at(i).x > thumbTip.x) ||
				(handDirection == 1 && fingers.at(i).x < thumbTip.x))
				tmp.push_back(fingers.at(i));
		}

		fingers = tmp;
		
		ofstream myfile;
		if(saveToFile)
		{
			myfile.open ("hand.txt",ios::out | ios::app);
			myfile << "----------------------------------------.\n";
		}

		if (!saveToFile || myfile.is_open())
		{
			int fingersFound = 1;
			ofVec3f currentFinger = vCentroid - thumbTip;
			ofVec3f currentFingerTip = thumbTip;

			if(saveToFile)
				myfile << "thumb length: " << currentFinger.length() << endl;

			//Largo del pulgar
			if(currentFinger.length() > MIN_LENGTH_FINGERS.at(0) && currentFinger.length() < MAX_LENGTH_FINGERS.at(0))
				prob += .111;
			else
				cout << "Largo de pulgar deficiente: " << currentFinger.length()  << " - (" << MIN_LENGTH_FINGERS.at(0) << ", " << MAX_LENGTH_FINGERS.at(0) << ")"<<endl;


			bool fingerFounded = true;
			while(fingerFounded)
			{
				int fingerIdx = findCloserFingerTo(currentFingerTip,fingers,handDirection);
				if(fingerIdx != -1)
				{
					currentFingerTip = fingers.at(fingerIdx);
					saveCloud("finger" + ofToString(fingersFound) + ".pcd", fingers.at(fingerIdx));
					ofVec3f newFinger = vCentroid - currentFingerTip;
					double angle = (acos(currentFinger.dot(newFinger)/(currentFinger.length()*newFinger.length()))*180)/PI;
					if(saveToFile)
						myfile << "finger" << fingersFound << "length: " << newFinger.length() << " - angle: " << angle << endl;
					
					if(fingersFound < 5)
					{
						//Chequeo angulo
						if(angle > MIN_ANGLES_FINGERS.at(fingersFound - 1) && angle < MAX_ANGLES_FINGERS.at(fingersFound - 1))
							prob += .111;
						else
							cout << "angulo " << fingersFound << " deficiente: " << angle << " - (" << MIN_ANGLES_FINGERS.at(fingersFound - 1) << ", " << MAX_ANGLES_FINGERS.at(fingersFound - 1) << ")"<<endl;
						//Chequeo largo
						if(newFinger.length() < MAX_LENGTH_FINGERS.at(fingersFound) && newFinger.length() > MIN_LENGTH_FINGERS.at(fingersFound))
							prob += .111;
						else
							cout << "largo " << fingersFound << " deficiente: " << newFinger.length() << " - (" << MIN_LENGTH_FINGERS.at(fingersFound) << ", " << MAX_LENGTH_FINGERS.at(fingersFound) << ")"<<endl;
					}

					currentFinger = newFinger;
					fingers.erase(fingers.begin() + fingerIdx);
					fingersFound++;
				}
				else
					fingerFounded = false;
			}

			if(saveToFile)
				myfile << "fingers: " << fingersFound << endl;
			
			cout << "Prob: " << prob << endl;
			myfile.close();

		//	if(fingersFound == 5 && prob > .80)
		//	{
		//		return true;
		//		//trackedClouds.push_back(TrackedCloud(clusterBack,true,true));
		//	}
		//	else
		//		return false;
		}
		return prob;
	}

	float HandDetector::IsHand()
	{
		if(!table->isOnTable(hand))
		{
			//Elimino puntos por fuera de la mesa
			//Busco el extremo de la mano
			//Determino la dirección de la mano
			trimPointsOutsideTable();
			//cout << "direccion: " << handDirection << endl;
			
			if(hand->size() > 0)
			{
				//Corto la mano... ouch!
				trimHand();

				//Busco los dedos
				vector<ofVec3f> fingers = getFingers();
			
				return checkFingers(fingers);
			}
			else
				return 0;

			
		}
		return 0;
	}

	int HandDetector::GetHandDirection()
	{
		return handDirection;
	}
}