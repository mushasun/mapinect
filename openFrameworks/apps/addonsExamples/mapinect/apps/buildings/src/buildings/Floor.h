#ifndef FLOOR_H__
#define FLOOR_H__

#include "IPolygon.h"
#include "ITxManager.h"
#include "ofGraphics.h"

using namespace mapinect;

namespace buildings {

	class Floor {
	public:
		Floor(const IPolygonPtr& modelObject)					{ this->modelObject = modelObject; }
		virtual ~Floor()										{ }

		inline const IPolygonPtr& getModelObject() const		{ return modelObject; }
		inline void updateModelObject(const IPolygonPtr& p)		{ modelObject = p; }

		virtual void	draw(const ITxManager* txManager);

		static GLuint	floorTexture;

	private:
		IPolygonPtr		modelObject;
	};
}

#endif	// FLOOR_H__
