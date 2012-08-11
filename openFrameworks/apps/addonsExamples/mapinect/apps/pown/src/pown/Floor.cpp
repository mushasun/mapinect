#include "Floor.h"

#include "ofGraphicsUtils.h"
#include "PownConstants.h"
#include "SoundManager.h"

namespace pown
{
	Floor::Floor(const IObjectPtr& object, const ofColor& color)
		: Box(object, color)
	{
	}

	Floor::~Floor()
	{
	}

	bool removeBumpEffectIfIsDead(const FloorBumpEffect& fbe)
	{
		return !fbe.isAlive();
	}

	void Floor::update(float elapsedTime)
	{
		Box::update(elapsedTime);

		for (list<FloorBumpEffect>::iterator fbe = bumpEffects.begin(); fbe != bumpEffects.end(); fbe++)
			fbe->update(elapsedTime);

		bumpEffects.remove_if(removeBumpEffectIfIsDead);
	}

	void Floor::draw() const
	{
		Box::draw();

		for (list<FloorBumpEffect>::const_iterator fbe = bumpEffects.begin(); fbe != bumpEffects.end(); fbe++)
			fbe->draw();
	}

	bool Floor::testHit(Bolt* bolt) const
	{
		IPolygonPtr polygon = getPolygon();
		ofVec3f projected(polygon->getMathModel().getPlane().project(bolt->getPosition()));
		bool result = !polygon->getMathModel().isInPolygon(projected);
		return result;
	}

	void Floor::absorbBolt(Bolt* bolt)
	{
		const ofVec3f& boltPosition(bolt->getPosition());
		const vector<Line3D>& edges(getPolygon()->getMathModel().getEdges());
		int closest = 0;
		int edgeCount = edges.size();
		for (int i = 0; i < edgeCount; i++)
			if (edges[i].distance(boltPosition) < edges[closest].distance(boltPosition))
				closest = i;
		
		SoundManager::playNote(closest);
		int edgeA = (closest + edgeCount - 1) % edgeCount;
		int edgeB = (closest + 1) % edgeCount;

		FloorBumpEffect fbe(
			bolt->getColor(),
			bolt->getIntensity(),
			Line3D(edges[edgeA].getDestination(), edges[edgeA].getOrigin()),
			edges[edgeB]);

		bumpEffects.push_back(fbe);

		bolt->absorb();
	}


#define BUMP_EFFECT_ALIVE_PERCENT				0.0f


	FloorBumpEffect::FloorBumpEffect(const ofColor& color, float widthPercent, const Line3D& lineA, const Line3D& lineB)
		: color(color), widthPercent(widthPercent), lifetime(1), lineA(lineA), lineB(lineB)
	{
	}

	void FloorBumpEffect::update(float elapsedTime)
	{
		lifetime -= elapsedTime / PownConstants::FLOOR_BUMP_EFFECT_LIFETIME;
		color.a = lifetime * 255.0f;
	}

	void FloorBumpEffect::draw() const
	{
		Line3D bar1(bar(0, 1));
		Line3D bar2(bar((0.5f - abs(lifetime - 0.5f)) * widthPercent * PownConstants::FLOOR_BUMP_EFFECT_WIDTH, 1));

		ofSetColor(color);
		ofDrawQuadTextured(bar1.getOrigin(), bar1.getDestination(), bar2.getDestination(), bar2.getOrigin());
	}

	Line3D FloorBumpEffect::bar(float advance, float length) const
	{
		Line3D barLine(lineA.calculateValue(advance), lineB.calculateValue(advance));
		float spacing = (1 - length) * 0.5;
		ofVec3f pt1(barLine.calculateValue(spacing));
		pt1.y -= 0.001f;
		ofVec3f pt2(barLine.calculateValue(1 - spacing));
		pt2.y -= 0.001f;
		Line3D result(pt1, pt2);
		return result;
	}

	bool FloorBumpEffect::isAlive() const
	{
		return lifetime > BUMP_EFFECT_ALIVE_PERCENT;
	}
}
