#ifndef MAPINECT_TYPES_H__
#define MAPINECT_TYPES_H__

// boost
#include <boost/shared_ptr.hpp>

// pcl includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>

// openFrameworks includes
#include "ofVec3f.h"

// pcl short types
typedef pcl::PointCloud<pcl::PointXYZ>	PC;
typedef PC::Ptr							PCPtr;

// mapinect forward declarations
namespace mapinect
{
	class Table;
	
	typedef boost::shared_ptr<Table> TablePtr;
}

#endif	// MAPINECT_TYPES_H__