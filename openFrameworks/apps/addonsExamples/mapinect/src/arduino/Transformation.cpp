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
		isWorldTransformationStable = true;
	}

	const Eigen::Affine3f	Transformation::getWorldTransformation() const
	{
		{
			transformationMatrixMutex.lock();
			Eigen::Affine3f transf (worldTransformation);
			transformationMatrixMutex.unlock();
			return transf;
		}
	}

	const Eigen::Affine3f	Transformation::getInverseWorldTransformation() const
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

	const Eigen::Affine3f Transformation::getInitialWorldTransformation() const
	{
			transformationMatrixMutex.lock();
			Eigen::Affine3f transf (initialWorldTransformation);
			transformationMatrixMutex.unlock();
			return transf;

	}

	void Transformation::setInitialWorldTransformation(const Eigen::Affine3f& newTransformation)
	{
			transformationMatrixMutex.lock();
			initialWorldTransformation = newTransformation;
			transformationMatrixMutex.unlock();
	}


	bool Transformation::getIsWorldTransformationStable() const
	{
		return isWorldTransformationStable;
	}
	
	void Transformation::setIsWorldTransformationStable(bool isStable)
	{
		isWorldTransformationStable = isStable;
	}

}
