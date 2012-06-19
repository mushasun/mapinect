#ifndef BUILDING_H__
#define BUILDING_H__

#include "ITxManager.h"
#include "IObject.h"
#include "Floor.h"
#include "Visualizer.h"

using namespace mapinect;

namespace visualizer {

	class Box {
	public:
		Box(const IObjectPtr& object);
		virtual ~Box();

		void updateModelObject(const IObjectPtr& ob) { object = ob; }

		void update(float progress);
		void draw(const ITxManager* txManager, const Floor& floor);

		//inline void setProgress(float p) { progress = p;}
	private:
		float			progress;
		IObjectPtr		object;

		static ofImage* barTexture;
		static ofImage* roofTexture;
		Visualizer		vis;
	};
}

#endif	// BUILDING_H__
