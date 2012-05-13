#include "Grab.h"

namespace photo {
	Grab::Grab(Photo* photo, PCHand* hand)
	{
		this->photo = photo;
		this->hands.push_back(hand);
	}
			
	bool Grab::Update(PCHand* hand)
	{
		int id = hand->getId();
		for (list<PCHand*>::iterator handIter = hands.begin(); handIter != hands.end(); handIter++) {
			if((*handIter)->getId() == id)
			{
				photo->setPos((*handIter)->getFingerTips().front());
				return true;
			}
		}
		return false;
	}

	void Grab::draw()
	{
		/*for (list<PCHand*>::iterator handIter = hands.begin(); handIter != hands.end(); handIter++) {
			(*handIter)->draw();
		}*/
	}
}