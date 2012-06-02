#ifndef BUILDING_H__
#define BUILDING_H__

#include "ITxManager.h"
#include "IObject.h"
#include "Floor.h"

using namespace mapinect;

namespace buildings {

	class Building {
	public:
		Building(const IObjectPtr& object);
		virtual ~Building();

		void updateModelObject(const IObjectPtr& ob) { object = ob; }

		void update();
		void draw(const ITxManager* txManager, const Floor& floor);

		static GLuint	buildingTexture;
		static GLuint	roofTexture;

	private:
		float			progress;
		IObjectPtr		object;
	};
}

#endif	// BUILDING_H__
