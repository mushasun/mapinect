#ifndef MAPINECT_PC_MODEL_OBJECT_H__
#define MAPINECT_PC_MODEL_OBJECT_H__

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include "ModelObject.h"

#include "ofVec3f.h"
#include "mapinectTypes.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include "ofGraphicsUtils.h"

using namespace pcl;

namespace mapinect {
	class PCModelObject : public ModelObject {
	public:
		PCModelObject(const PCPtr& cloud, int objId = -1);
		virtual ~PCModelObject();

		virtual void draw();

		inline void					setTransformation (Eigen::Affine3f *_transformation) { transformation = *_transformation ;}
		inline void					setCloud (const PCPtr& nuCloud)		{ cloud = nuCloud ;}
		inline void					setDrawPointCloud(bool draw)		{ drawPointCloud = draw; }
		inline const PCPtr&			getCloud()							{ return cloud; }
		inline bool					hasObject()							{ return modelObject != NULL; }
		inline int					getLod()							{ return lod; }
		virtual void				resetLod();
		inline ofVec3f				getvMin()							{ return vMin; }
		inline ofVec3f				getvMax()							{ return vMax; }

		virtual void				addToModel(const PCPtr& nuCloud);
		void						updateCloud(const PCPtr& nuCloud);

		virtual void				detectPrimitives();
		virtual void				increaseLod();

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
	protected:
		bool						drawPointCloud;

		ofVec3f						vMin;
		ofVec3f						vMax;
		Eigen::Affine3f				transformation;
		PCPtr						cloud;
		ModelObject*				modelObject;
		int							lod;

		virtual void				applyTransformation();

	};
}
#endif	// MAPINECT_PC_MODEL_OBJECT_H__
