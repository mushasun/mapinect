#ifndef MAPINECT_PC_MODEL_OBJECT_H__
#define MAPINECT_PC_MODEL_OBJECT_H__

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include "ModelObject.h"

#include "mapinectTypes.h"
#include "Polygon3D.h"

namespace mapinect {

	class PCModelObject;

	typedef boost::shared_ptr<PCModelObject> PCModelObjectPtr;

	class PCModelObject : public ModelObject {
	public:
		PCModelObject(const PCPtr& cloud, int objId = -1);
		virtual ~PCModelObject();

		virtual void draw();

		inline void					setTransformation (Eigen::Affine3f *_transformation) { transformation = *_transformation ;}
		inline void					setCloud (const PCPtr& nuCloud)		{ cloud = nuCloud ;}
		virtual void				setAndUpdateCloud (const PCPtr& cloud);
		void						setDrawPointCloud(bool draw);
		inline const PCPtr&			getCloud()							{ return cloud; }
		inline bool					hasObject()							{ return modelObject != NULL; }
		inline int					getLod()							{ return lod; }
		virtual void				resetLod();
		inline ofVec3f				getvMin()							{ return vMin; }
		inline ofVec3f				getvMax()							{ return vMax; }

		virtual vector<Polygon3D>	getMathModelApproximation() const = 0;

		virtual void				addToModel(const PCPtr& nuCloud);
		virtual void				detectPrimitives();
		virtual void				increaseLod(const PCPtr& nuCloud);

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
	protected:
		bool					drawPointCloud;

		ofVec3f						vMin;
		ofVec3f						vMax;
		Eigen::Affine3f				transformation;
		PCPtr						cloud;
		ModelObjectPtr				modelObject;
		int							lod;

		virtual void			applyTransformation();

	};
}
#endif	// MAPINECT_PC_MODEL_OBJECT_H__
