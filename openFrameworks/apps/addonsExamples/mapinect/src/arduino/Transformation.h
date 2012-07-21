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
		Eigen::Affine3f							getWorldTransformation() const;
		Eigen::Affine3f							getInverseWorldTransformation() const;
		void									setWorldTransformation(const Eigen::Affine3f& newTransformation);

		mutable ofxMutex						transformationMatrixMutex;

		mutable ofxMutex						cloudMutex;

		static  Eigen::Affine3f							initialWorldTransformation;
	private:
		static Eigen::Affine3f							worldTransformation;
		static Eigen::Affine3f							inverseWorldTransformation;


	};
}

#endif	// MAPINECT_TRANSFORMATION_MATRIX_H__
