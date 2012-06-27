#ifndef MAPINECT_PC_MODEL_OBJECT_H__
#define MAPINECT_PC_MODEL_OBJECT_H__

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include "ModelObject.h"

#include "IObject.h"
#include "mapinectTypes.h"

namespace mapinect {

	class PCModelObject;

	typedef boost::shared_ptr<PCModelObject> PCModelObjectPtr;

	class PCModelObject : public ModelObject {
	public:
		PCModelObject(const PCPtr& cloud, int objId = -1);
		virtual ~PCModelObject();

		virtual IObjectPtr			getMathModelApproximation() const;

		virtual void				draw();

		inline void					setTransformation (Eigen::Affine3f *_transformation) { transformation = *_transformation ;}
		inline void					setCloud (const PCPtr& nuCloud)		{ cloud = nuCloud ;}
		virtual void				setAndUpdateCloud (const PCPtr& cloud);
		void						setDrawPointCloud(bool draw);

		inline const PCPtr&			getCloud() const					{ return cloud; }
		inline bool					hasObject()	const					{ return modelObject != NULL; }
		inline int					getLod() const						{ return lod; }
		virtual void				resetLod();
		inline ofVec3f				getvMin() const						{ return vMin; }
		inline ofVec3f				getvMax() const						{ return vMax; }

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
