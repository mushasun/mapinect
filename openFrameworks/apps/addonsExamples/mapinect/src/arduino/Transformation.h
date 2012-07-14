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
		void									setWorldTransformation(const Eigen::Affine3f& newTransformation);

		mutable ofxMutex						transformationMatrixMutex;

		mutable ofxMutex						cloudMutex;

	private:
		Eigen::Affine3f							worldTransformation;

	};
}

#endif	// MAPINECT_TRANSFORMATION_MATRIX_H__
