#ifndef FLOOR_H__
#define FLOOR_H__

#include "ofGraphics.h"
#include "PCPolygon.h"
#include "ITxManager.h"

using namespace mapinect;

namespace buildings {

	class Floor {
	public:
		Floor(const TablePtr& modelObject)				{ this->modelObject = modelObject; }
		virtual ~Floor()						{ }

		inline const TablePtr& getModelObject() const { return modelObject; }

		virtual void	draw(const ITxManager* txManager);

		static GLuint	floorTexture;

	private:
		TablePtr		modelObject;
	};
}

#endif	// FLOOR_H__
