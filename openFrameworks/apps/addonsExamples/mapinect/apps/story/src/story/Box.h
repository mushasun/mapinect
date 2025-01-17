#ifndef BOX_H__
#define BOX_H__

#include "IObject.h"
#include "ofImage.h"
#include "IButtonManager.h"
#include "DataTouch.h"
#include "AnimatedSprite.h"
#include "ofSoundPlayer.h"

using namespace mapinect;

namespace story
{
	enum BuildType{
		kHouse,
		kPowerPlant,
		kWaterPlant
	};

	class Box
	{
	public:
		Box(const IObjectPtr& object, IButtonManager* btnManager);
		virtual ~Box();

		static void				setup();

		void					updateModelObject(const IObjectPtr& ob);		

		virtual void			update(float elapsedTime);
		virtual void			draw();
		virtual void			burn();
		virtual void			stopBurn();

		inline vector<int>		getButtonsId()		{ return buttonsId; }
		inline BuildType		getBuildType()		{ return buildType; }
		
		virtual void			buttonEvent(const IButtonPtr& btn, bool released) = 0;
		virtual void			objectEvent(const DataTouch& touchPoint, const BuildType& selection) = 0;
		ofSoundPlayer*			burnSound;

	protected:
		IObjectPtr				object;
		ofImage*				textureTop;
		ofImage*				textureA;
		ofImage*				textureB;
		ofImage*				textureC;
		ofImage*				textureD;
		vector<int>				buttonsId;
		BuildType				buildType;
		int						objectID;		
		bool					isBurning;

		AnimatedSprite*	fireSpriteA;
		AnimatedSprite*	fireSpriteB;
		AnimatedSprite*	fireSpriteC;
		AnimatedSprite*	fireSpriteD;
		
		static vector<ofImage*> fireImgs;
	};
}

#endif	// BOX_H__
