#ifndef MAPINECT_PC_MODEL_OBJECT_H__
#define MAPINECT_PC_MODEL_OBJECT_H__

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include "ModelObject.h"

#include <pcl/point_types.h>
#include "ofxVec3f.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include "ofxKinect.h"
#include "ofGraphicsUtils.h"

using namespace pcl;

namespace mapinect {
	class PCModelObject : public ModelObject {
	public:
		PCModelObject();
		PCModelObject(PointCloud<PointXYZ>::Ptr cloud, PointCloud<PointXYZ>::Ptr extendedCloud);
		PCModelObject(PointCloud<PointXYZ>::Ptr cloud, PointCloud<PointXYZ>::Ptr extendedCloud, int objId);
		virtual ~PCModelObject();

		virtual void draw();

		inline void setTransformation (Eigen::Affine3f *_transformation) { transformation = *_transformation ;}
		inline void setCloud (PointCloud<PointXYZ>::Ptr nuCloud) { cloud = *nuCloud ;}
		virtual void applyTransformation();
		inline void						setDrawPointCloud(bool draw)		{ drawPointCloud = draw; }
		inline PointCloud<PointXYZ>		getCloud()							{ return cloud; }
		void							updateCloud(PointCloud<PointXYZ>::Ptr nuCloud);

		virtual void				detectPrimitives();

	protected:
		bool						drawPointCloud;

		ofxVec3f					vMin;
		ofxVec3f					vMax;
		Eigen::Affine3f				transformation;
		PointCloud<PointXYZ>		cloud;
		PointCloud<PointXYZ>		extendedcloud;
		ModelObject*				modelObject;
		int id;
	};
}
#endif	// MAPINECT_PC_MODEL_OBJECT_H__