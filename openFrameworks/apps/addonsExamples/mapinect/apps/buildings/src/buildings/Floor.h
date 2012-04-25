#ifndef FLOOR_H__
#define FLOOR_H__

#include "ofGraphics.h"
#include "PCPolygon.h"
#include "ITxManager.h"

using namespace mapinect;

namespace buildings {

	class Floor {
	public:
		Floor(PCPolygon* modelObject)			{ this->modelObject = modelObject; }
		virtual ~Floor()						{ }

		inline const PCPolygon* getModelObject() const { return modelObject; }

		virtual void	draw(const ITxManager* txManager);

		static GLuint	floorTexture;

	private:
		PCPolygon*		modelObject;
	};
}

#endif	// FLOOR_H__
