#include "Transformation.h"

namespace mapinect
{
	Eigen::Affine3f Transformation::worldTransformation;
	Eigen::Affine3f Transformation::inverseWorldTransformation;
	Eigen::Affine3f Transformation::initialWorldTransformation;

	Transformation::Transformation() 
	{
		worldTransformation  = Eigen::Affine3f::Identity();
		initialWorldTransformation = Eigen::Affine3f::Identity();
	}

	Eigen::Affine3f	Transformation::getWorldTransformation() const
	{
		{
			transformationMatrixMutex.lock();
			Eigen::Affine3f transf (worldTransformation);
			transformationMatrixMutex.unlock();
			return transf;
		}
	}

	Eigen::Affine3f	Transformation::getInverseWorldTransformation() const
	{
		{
			transformationMatrixMutex.lock();
			Eigen::Affine3f transf (inverseWorldTransformation);
			transformationMatrixMutex.unlock();
			return transf;
		}
	}


	void Transformation::setWorldTransformation(const Eigen::Affine3f& newTransformation)
	{
		{
			transformationMatrixMutex.lock();
			worldTransformation = newTransformation;
			inverseWorldTransformation = newTransformation.inverse();
			transformationMatrixMutex.unlock();
		}
	}

}
