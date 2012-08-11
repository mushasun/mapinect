#ifndef MAPINECT_TRANSFORMATION_MATRIX_H__
#define MAPINECT_TRANSFORMATION_MATRIX_H__

#include "ofxMutex.h"
#include <Eigen/Geometry>

namespace mapinect
{
	class Transformation
	{
	public:
		Transformation();

		// thread-safe
		const Eigen::Affine3f					getWorldTransformation() const;
		const Eigen::Affine3f					getInverseWorldTransformation() const;
		void									setWorldTransformation(const Eigen::Affine3f& newTransformation);
		const Eigen::Affine3f					getInitialWorldTransformation() const;
		void									setInitialWorldTransformation(const Eigen::Affine3f& newTransformation);
		mutable ofxMutex						transformationMatrixMutex;

		mutable ofxMutex						cloudMutex;

		bool									getIsWorldTransformationStable() const;
		void									setIsWorldTransformationStable(bool isStable); 

	private:
		bool									isWorldTransformationStable;
		static Eigen::Affine3f					initialWorldTransformation;

		static Eigen::Affine3f					worldTransformation;
		static Eigen::Affine3f					inverseWorldTransformation;


	};
}

#endif	// MAPINECT_TRANSFORMATION_MATRIX_H__
