#ifndef BOMBEROS_H__
#define BOMBEROS_H__

#include "IObject.h"
#include "IButton.h"
#include "../Box.h"
#include <map>
#include "ofImage.h"
#include "ofMain.h"


using namespace mapinect;

namespace story
{
	class Bomberos : public Box
	{
		public:
			Bomberos(const IObjectPtr& object, IButtonManager* btnManager);
			//virtual ~House(); 
			
			static void		setup();
			
			virtual void buttonEvent(const IButtonPtr& btn, bool released);
			virtual void objectEvent(const DataTouch& touchPoint, const BuildType& selection);

			virtual void			update(float elapsedTime);
			virtual void			draw();

		private:
			static void				loadTextures();
			void					associateTextures();
			static void				loadSounds();

			//control
			bool					isWatering;
			IButtonManager*			btnManager;
			float					timeWatering;

			//sounds
			static ofSoundPlayer*	water;
			static ofSoundPlayer*	sirena;

			//textures
			static ofImage*		txTruckTop;
			static ofImage*		txTruckSide;
	};
}

#endif	// HOUSE_H__
