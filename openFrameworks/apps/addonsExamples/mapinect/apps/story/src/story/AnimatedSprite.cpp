#include "AnimatedSprite.h"
#include "ofGraphicsUtils.h"

namespace story {
	AnimatedSprite::AnimatedSprite(vector<ofImage*> images, float frameRate, Polygon3D pol):sprites(images),frameRate(frameRate)
	{
		time = 0;
		spritesCount = sprites.size();
		assert(spritesCount > 0);
		idx = 0;

		setPolygon(pol);	
	}

	void AnimatedSprite::setPolygon(Polygon3D pol)
	{
		ofVec3f polNormal = pol.getPlane().getNormal();
		ofVec3f translate = polNormal * 0.001;
		vector<ofVec3f> oldVexs = pol.getVertexs();
		vector<ofVec3f> newVexs;
		for(int i = 0; i < 4; i++)
			newVexs.push_back(oldVexs.at(i)+translate);
		polygon.setVertexs(newVexs);
	}

	
	void AnimatedSprite::update(float elapsed)
	{
		time += elapsed;
		if(time > frameRate)
		{
			time = 0;
			idx = (idx + 1) % spritesCount;
		}
	}
			
	void AnimatedSprite::draw()
	{
		ofImage* current = sprites.at(idx);

		current->bind();
		ofDrawQuadTextured(polygon.getVertexs(), ofTexCoordsFor());
		current->unbind();
	}

}