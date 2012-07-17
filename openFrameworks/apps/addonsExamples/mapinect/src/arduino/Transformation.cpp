#include "Transformation.h"

namespace mapinect
{
	Eigen::Affine3f Transformation::worldTransformation;

	Transformation::Transformation() 
	{
		worldTransformation  = Eigen::Affine3f::Identity();
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

	void Transformation::setWorldTransformation(const Eigen::Affine3f& newTransformation)
	{
		{
			transformationMatrixMutex.lock();
			worldTransformation = newTransformation;
			transformationMatrixMutex.unlock();
		}
	}

}
