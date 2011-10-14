#include "PCMThread.h"

#include "ofMain.h"
#include "pointUtils.h"
#include "ofxVecUtils.h"
#include "ofGraphicsUtils.h"
#include "ofxXmlSettings.h"
#include "utils.h"

using namespace std;

#define DEFAULT_NAME		"test"
#define WAIT_TIME_MS		40

void PCMThread::setup() {
	//Cargo archivo de config.
	ofxXmlSettings XML;
	if(XML.loadFile("PCM_Config.xml")){
	
		OCTREE_RES = XML.getValue("PCMConfig:OCTREE_RES", 0.1);
		MIN_DIFF_TO_PROCESS = XML.getValue("PCMConfig:MIN_DIFF_TO_PROCESS", 40);
		QUAD_HALO = XML.getValue("PCMConfig:QUAD_HALO", 10);
		DIFF_THRESHOLD = XML.getValue("PCMConfig:DIFF_THRESHOLD", 20);
		RES_IN_OBJ = XML.getValue("PCMConfig:RES_IN_OBJ", 0.001);
		DIFF_IN_OBJ = XML.getValue("PCMConfig:DIFF_IN_OBJ", 20);
		TIMES_TO_CREATE_OBJ = XML.getValue("PCMConfig:TIMES_TO_CREATE_OBJ", 5);
	}

	// Initialize point cloud
	cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
		(new pcl::PointCloud<pcl::PointXYZ>()); 
	cloud->width    = KINECT_WIDTH;
	cloud->height   = KINECT_HEIGHT;
	cloud->is_dense = false;
	cloud->points.resize (CLOUD_POINTS);

	// Inicializo octree
	octree = new octree::OctreePointCloudChangeDetector<PointXYZ>(OCTREE_RES); //Valor de resolucion sacado del ejemplo

	timer = 0;
	baseCloudSetted = false;
	noDifferencesCount = 0;
	detectMode = false;

	startThread(true, false);

}

//--------------------------------------------------------------
void PCMThread::threadedFunction() {
	while (isThreadRunning()) {
		if (lock()) {

			if(!baseCloudSetted && gKinect->isFrameNew()) {
				setInitialPointCloud();
				baseCloudSetted = true;
			}

			if(detectMode)
				processDiferencesClouds();

			unlock();
			ofSleepMillis(WAIT_TIME_MS);
		}
	}
}

PointCloud<PointXYZ>::Ptr PCMThread::getCloud(){
	return getPartialCloudRealCoords(ofPoint(0,0),ofPoint(KINECT_WIDTH,KINECT_HEIGHT));
}

PointCloud<PointXYZ>::Ptr PCMThread::getPartialCloud(ofPoint min, ofPoint max){
	//Calcular tamaño de la nube
	PointCloud<PointXYZ>::Ptr partialColud (new pcl::PointCloud<pcl::PointXYZ>);
	partialColud->width    = max.x - min.x;
	partialColud->height   = max.y - min.y;
	partialColud->is_dense = false;
	partialColud->points.resize (partialColud->width*partialColud->height);

	//Recorrer el mapa de distancias obteniendo sólo los que estén dentro del rectángulo

	register int centerX = (cloud->width >> 1);
	int centerY = (cloud->height >> 1);
	float bad_point = std::numeric_limits<float>::quiet_NaN();

	register float* depth_map = gKinect->getDistancePixels();

	int step = 1;
	register int depth_idx = 0;
	int absV, absU, cloud_idx = 0;
	for(int v = -centerY; v < centerY; ++v) {
		for(register int u = -centerX; u < centerX; ++u, ++depth_idx) {
			absV = v + centerY;
			absU = u + centerX;
			if( absV > min.y && absV < max.y && absU > min.x && absU < max.x)
			{
				pcl::PointXYZ& pt = partialColud->points[cloud_idx];

				// Check for invalid measurements o puntos que estan fuera del nearthreshold/farthreshold
				// TODO: limitar que depth_map[depth_idx] caiga dentro de nearThreshold/farThreshold
				// Lo intente hacer pero parece que esos valores no corresponden con lo que lee el kinect
				if(/*depth_map[depth_idx] > 100 || */depth_map[depth_idx] == 0){
					pt.x = pt.y = pt.z = 0;
					//continue;
				}
				else{
					//ofxVec3f point = kinect->getWorldCoordinateFor(u,v);
					
					pt.z = depth_map[depth_idx];
					pt.x = u;
					pt.y = v;

				}
				cloud_idx++;
			}
		}
	}
	//pcl::io::savePCDFileASCII ("test_pacial_pcd.pcd", *partialColud);
	return partialColud;	
}

PointCloud<PointXYZ>::Ptr PCMThread::getPartialCloudRealCoords(ofPoint min, ofPoint max, int density){
	//Chequeo
	if (min.x < 0 || min.y < 0 || max.x > KINECT_WIDTH || max.y > KINECT_HEIGHT || min.x > max.x || min.y > max.y) //* load the file
	{
		PCL_ERROR ("Error en parametros de entrada para obtener la nube \n");
	}
	
	//Calcular tamaño de la nube
	PointCloud<PointXYZ>::Ptr partialColud (new pcl::PointCloud<pcl::PointXYZ>);
	partialColud->width    = ceil((max.x - min.x)/density);
	partialColud->height   = ceil((max.y - min.y)/density);
	partialColud->is_dense = false;
	partialColud->points.resize (partialColud->width*partialColud->height);
	register float* depth_map = gKinect->getDistancePixels();
	//Recorrer el mapa de distancias obteniendo sólo los que estén dentro del rectángulo
	register int depth_idx = 0;
	int cloud_idx = 0;
	for(int v = min.y; v < max.y; v += density) {
		for(register int u = min.x; u < max.x; u += density) {
			pcl::PointXYZ& pt = partialColud->points[cloud_idx];
			cloud_idx++;
			
			// Check for invalid measurements
			if(depth_map[depth_idx] == 0){
				pt.x = pt.y = pt.z = 0;
			}
			else{
				ofxVec3f pto = gKinect->getWorldCoordinateFor(u,v);
				pt.x = pto.x;
				pt.y = pto.y;
				pt.z = pto.z;
			}

			depth_idx += density;
		}
	}
	//pcl::io::savePCDFileASCII ("partial_real.pcd", *partialColud);
	return partialColud;	
}

//--------------------------------------------------------------
void PCMThread::setInitialPointCloud() {
	cloud = getCloud();
	currentDiffcloud = cloud;
}

PointCloud<PointXYZ>::Ptr PCMThread::getDifferenceIdx(bool &isDif, int noise_filter) {
	PointCloud<PointXYZ>::Ptr difCloud (new PointCloud<PointXYZ>());
	PointCloud<PointXYZ>::Ptr secondCloud = getCloud();
	isDif = false;

	/*pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ> ("base.pcd", *currentDiffcloud, false);
	writer.write<pcl::PointXYZ> ("dif.pcd", *secondCloud, false);
*/

	int dif = getDifferencesCloud(currentDiffcloud,secondCloud,difCloud,OCTREE_RES);
	if(dif > DIFF_THRESHOLD || noDifferencesCount < 25){
		//cout << dif << endl;
		if(dif < DIFF_THRESHOLD)
			noDifferencesCount++;
		else
			noDifferencesCount = 0;

		currentDiffcloud = secondCloud;
		dif = getDifferencesCloud(cloud,secondCloud,difCloud,OCTREE_RES);
		if(dif  > MIN_DIFF_TO_PROCESS){
			isDif = true;
		}
	}

	return difCloud;
}

bool countIsZero(const TrackedCloud &trackedCloud) {
	return ((TrackedCloud)trackedCloud).getCounter() == 0;
}

void PCMThread::processDiferencesClouds() {
	bool dif;
	//nDetectedObjects = openCVService->contourFinder.blobs.size();
	PointCloud<PointXYZ>::Ptr filteredCloud = getDifferenceIdx(dif);
	if (dif) {
		
		//Subdivido la nube de diferencias en clusters
		// Creating the KdTree object for the search method of the extraction
		pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
		tree->setInputCloud (filteredCloud);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance (0.02); // 2cm
		ec.setMinClusterSize (100);
		ec.setMaxClusterSize (25000);
		ec.setSearchMethod (tree);
		ec.setInputCloud(filteredCloud);
		ec.extract (cluster_indices);

		/*nDetectedObjects = cluster_indices.size();
		int j = 0;
*/

		//Actualizo las detecciones temporales
		for (list<TrackedCloud>::iterator iter = trackedClouds.begin(); iter != trackedClouds.end(); iter++) {
			iter->addCounter(-1);
		}
		trackedClouds.remove_if(countIsZero);

		//separo en clusters y valido si existen nuevos objetos o actualizo ya detectados
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
				cloud_cluster->points.push_back (filteredCloud->points[*pit]); //*

			if(!updateDetectedObject(cloud_cluster)) {
				trackedClouds.push_back(TrackedCloud(cloud_cluster));
			}
			/*detectedObjects[j] = Object3D(cloud_cluster,cloud_cluster,kinect);
			j++;*/
		}
	}
}

//--------------------------------------------------------------
void PCMThread::saveCloud(const string& name){
	cout << "saving: " << name << "..." << endl;

	register int centerX = (cloud->width >> 1);
	int centerY = (cloud->height >> 1);
	float bad_point = std::numeric_limits<float>::quiet_NaN();

	register float* depth_map = gKinect->getDistancePixels();
	float constant = 0.5f; //CAlculado segun lo que hay en la libreria de PCL
	int step = 1;
	register int depth_idx = 0;
	for(int v = -centerY; v < centerY; ++v) {
		for(register int u = -centerX; u < centerX; ++u, ++depth_idx) {
			pcl::PointXYZ& pt = cloud->points[depth_idx];
			// Check for invalid measurements
			if (depth_map[depth_idx] == 0)
			{
				// not valid
				pt.x = pt.y = pt.z = bad_point;
				continue;
			}
			pt.z = depth_map[depth_idx];
			pt.x = u * constant;
			pt.y = v * constant;
			//pt.x = u * pt.z * constant;
			//pt.y = v * pt.z * constant;
		}
	}
	//pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	/*PointCloud<PointXYZ>::Ptr cloud_source_ptr; 

	cloud_source_ptr = cloud->makeShared(); */
	//viewer.showCloud (cloud);

	string filename = name + PCD_EXTENSION;
	pcl::io::savePCDFileASCII (filename, *cloud);
	std::cerr << "Saved " << cloud->points.size () << " data points to " << filename << std::endl;
}

void PCMThread::savePartialCloud(ofPoint min, ofPoint max, int id, const string& name){
	//Calcular tamaño de la nube
	cout << "saving: " << name << "..." << endl;
	PointCloud<PointXYZ>::Ptr partialColud = getPartialCloud(min,max);
	std::stringstream ss;
	ss << name << "-" << id << PCD_EXTENSION;
	pcl::io::savePCDFileASCII (ss.str(), *partialColud);
	std::cerr << "Saved " << partialColud->points.size () << " data points to " << ss.str() << std::endl;
}

PointCloud<PointXYZ>* PCMThread::loadCloud(const string& name) {
	cout << "loading: " << name << "..." << endl;
	pcl::PointCloud<pcl::PointXYZ>* tmpCloud = new pcl::PointCloud<PointXYZ>();
	string filename = name + PCD_EXTENSION;
	pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *tmpCloud);
	cout << name << " loaded!" << endl;
	return tmpCloud;
}

bool PCMThread::updateDetectedObject(PointCloud<PointXYZ>::Ptr cloud_cluster) {
	for (list<TrackedCloud>::iterator iter = trackedClouds.begin(); iter != trackedClouds.end(); iter++) {
		if (iter->matches(cloud_cluster))
		{
			if (!(iter->hasObject())) {
				iter->addCounter(2);
			}
			else {
				iter->addCounter(1);
			}
		}
	}

	return false;
}

//--------------------------------------------------------------
ofxVec3f PCMThread::normalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr plane){
	//Calculo de normales
		//Random indices
	int sampleSize = floor (plane->points.size() / 10.0f);
	std::vector<int> indices (sampleSize);
	for (size_t i = 0; i < sampleSize; i++) 
		indices[i] = rand() % plane->points.size();

	pcl::PointIndices::Ptr indicesptr (new pcl::PointIndices ());
	indicesptr->indices = indices;


	return normalEstimation(plane, indicesptr);
}

//--------------------------------------------------------------
ofxVec3f PCMThread::normalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr plane, pcl::PointIndices::Ptr indicesptr){
	//Calculo de normales
	int sampleSize = indicesptr->indices.size();
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (plane);
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);
	
	ne.setIndices(indicesptr);
	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (0.3);

	// Compute the features
	ne.compute (*cloud_normals);

	ofxVec3f result(0,0,0);
	//Promedio las normales
	float bad_point = std::numeric_limits<float>::quiet_NaN();
	for(int i = 0; i < sampleSize ; i++){
		pcl::Normal normal = cloud_normals->at(i);
		if (normal.normal_z != bad_point && normal.normal_z == normal.normal_z)
			result += PCLNORMAL_OFXVEC3F(cloud_normals->at(i));
	}
	
	result /= sampleSize;
	result = result.normalize();
	return result;
}
