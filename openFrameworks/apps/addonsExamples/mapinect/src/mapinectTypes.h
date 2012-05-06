#ifndef MAPINECT_TYPES_H__
#define MAPINECT_TYPES_H__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ>	PC;
typedef PC::Ptr							PCPtr;

namespace mapinect
{
	class Table;
}

#endif	// MAPINECT_TYPES_H__