#ifndef BUILDING_H__
#define BUILDING_H__

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
		void draw(const Floor& floor);

	private:
		float			progress;
		IObjectPtr		object;

		static ofImage* buildingTexture;
		static ofImage* roofTexture;

	};
}

#endif	// BUILDING_H__
