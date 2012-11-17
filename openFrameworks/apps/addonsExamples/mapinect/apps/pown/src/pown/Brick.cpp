#include "Brick.h"

#include "ofGraphicsUtils.h"
#include "PownConstants.h"

namespace pown
{
	const ofFloatColor	kDefaultBrickColor = kRGBDarkGray;
	const ofFloatColor	kBeatBoostColor = kRGBMidGray;
	const float			kWaveBaseIntensity = 0.8f;

	Brick::Brick(const NoteBeat& noteBeat, const Polygon3D& hitPolygon,
		const vector<ofVec3f>& drawVertexs, const ofFloatColor& color)
		: noteBeat(noteBeat), hitPolygon(hitPolygon), drawVertexs(drawVertexs), color(color)
	{
		center = computeCentroid(drawVertexs);
		normal = computeNormal(drawVertexs).normalized();
	}

	void Brick::update(float elapsedTime)
	{
	}

	void Brick::draw() const
	{
		ofSetColor(color);
		ofDrawQuad(drawVertexs);
	}

	Wave::Wave(const NoteBeat& noteBeat, const ofFloatColor& color, float intensity, float intensitySpeed, float radiusSpeed)
		: noteBeat(noteBeat), color(color), intensity(intensity), intensitySpeed(intensitySpeed),
		radius(0), radiusSpeed(radiusSpeed), radiusTimer(0)
	{
	}

	void Wave::update(float elapsedTime)
	{
		intensity -= intensitySpeed * elapsedTime;
		radiusTimer += elapsedTime;
		if (radiusTimer >= radiusSpeed)
		{
			radius++;
			radiusTimer -= radiusSpeed;
		}
	}

	set<NoteBeat> Wave::getNoteBeats() const
	{
		set<NoteBeat> overflowNoteBeats;
		int b = noteBeat.beat;
		for (int n = noteBeat.note - radius, i = 0; n <= noteBeat.note; n++, i++)
		{
			overflowNoteBeats.insert(NoteBeat(n, b - i));
			overflowNoteBeats.insert(NoteBeat(n, b + i));
		}
		for (int n = noteBeat.note + radius, i = 0; n > noteBeat.note; n--, i++)
		{
			overflowNoteBeats.insert(NoteBeat(n, b - i));
			overflowNoteBeats.insert(NoteBeat(n, b + i));
		}

		set<NoteBeat> result;
		for (set<NoteBeat>::iterator nb = overflowNoteBeats.begin(); nb != overflowNoteBeats.end(); nb++)
		{
			if (nb->inRange())
				result.insert(*nb);
			else if (PownConstants::WAVE_REBOUND)
			{
				int note = nb->note;
				if (!nb->inRangeNote())
					note = note < 0 ? abs(note) : PownConstants::NOTES - abs(note - PownConstants::NOTES + 1);
				int beat = nb->beat;
				if (!nb->inRangeBeat())
					beat = beat < 0 ? abs(beat) : PownConstants::BEATS - abs(beat - PownConstants::BEATS + 1);
				result.insert(NoteBeat(note, beat));
			}
		}

		return result;
	}

	bool Wave::isAlive() const
	{
		const float kWaveAlivePercent = 0.001;
		return intensity > kWaveAlivePercent;
	}

	ofFloatColor Wave::getColor() const
	{
		ofFloatColor result(color);
		result *= intensity;
		return result;
	}

	static int getBrickPos(const NoteBeat& noteBeat)
	{
		return noteBeat.note * PownConstants::BEATS + noteBeat.beat;
	}

	BrickManager::BrickManager(const IPolygonPtr& floor)
		: beat(0)
	{
		for (int i = 0; i < PownConstants::NOTES; i++)
			notesColors.push_back(ofRandomColor());

		const Polygon3D& floorPoly(floor->getMathModel());
		const Line3D beatsLine(floorPoly.getVertexs()[0], floorPoly.getVertexs()[1]);
		const Line3D notesLine(floorPoly.getVertexs()[0], floorPoly.getVertexs()[3]);
		const ofVec3f origin(notesLine.getOrigin());
		const ofVec3f dbeat(beatsLine.getDirection() / (float)PownConstants::BEATS);
		const ofVec3f dnote(notesLine.getDirection() / (float)PownConstants::NOTES);
		const float nudge = 0.025;
		const ofVec3f nbeat(dbeat * nudge);
		const ofVec3f nnote(dnote * nudge);

		bricks.resize(PownConstants::NOTES * PownConstants::BEATS);
		for (int i = 0; i < PownConstants::NOTES; i++)
		{
			for (int j = 0; j < PownConstants::BEATS; j++)
			{
				NoteBeat nb(i, j);
				vector<ofVec3f> hitVertexs;
				hitVertexs.push_back(origin + dnote * i		+ dbeat * j);
				hitVertexs.push_back(origin + dnote * (i+1)	+ dbeat * j);
				hitVertexs.push_back(origin + dnote * (i+1)	+ dbeat * (j+1));
				hitVertexs.push_back(origin + dnote * i		+ dbeat * (j+1));
				vector<ofVec3f> drawVertexs;
				drawVertexs.push_back(hitVertexs[0] + nnote + nbeat);
				drawVertexs.push_back(hitVertexs[1]	- nnote + nbeat);
				drawVertexs.push_back(hitVertexs[2]	- nnote - nbeat);
				drawVertexs.push_back(hitVertexs[3] + nnote - nbeat);
				bricks[getBrickPos(nb)] = new Brick(nb, Polygon3D(hitVertexs), drawVertexs, kDefaultBrickColor);
			}
		}
	}

	Brick* BrickManager::getBrick(int note, int beat) const
	{
		Brick* result = NULL;
		NoteBeat noteBeat(note, beat);
		if (noteBeat.inRange())
		{
			int pos = getBrickPos(noteBeat);
			result = bricks[pos];
		}
		return result;
	}

	void BrickManager::doBeat(map<int, Box*>& boxes)
	{
		beat++;
		if (beat == PownConstants::BEATS)
			beat = 0;

		for (map<int, Box*>::const_iterator box = boxes.begin(); box != boxes.end(); box++)
		{
			if (box->second->getNoteBeat().beat == beat)
			{
				beatBox(box->second);
			}
		}
	}

	void BrickManager::beatBox(Box* box)
	{
		box->doBeat();

		const float kWavesPerBeat = (float)PownConstants::WAVES_PER_BEAT;

		for (float i = 1.0f; i <= kWavesPerBeat; i += 1.0f)
			waves.push_back(new Wave(
				box->getNoteBeat(),
				box->getColor(),
				kWaveBaseIntensity * i / kWavesPerBeat,
				PownConstants::WAVE_INTENSITY_DECREASE_FACTOR * i,
				PownConstants::WAVE_RADIUS_INCREASE_TIME * i));
	}

	bool removeWaveIfNotAlive(Wave* wave)
	{
		bool result = !wave->isAlive();
		if (result)
			delete wave;
		return result;
	}

	void BrickManager::update(float elapsedTime)
	{
		for (vector<Brick*>::iterator brick = bricks.begin(); brick != bricks.end(); brick++)
			(*brick)->update(elapsedTime);
		for (list<Wave*>::iterator wave = waves.begin(); wave != waves.end(); wave++)
		{
			(*wave)->update(elapsedTime);
		}

		waves.remove_if(removeWaveIfNotAlive);
	}

	void BrickManager::draw(const Light& light)
	{
		map<Wave*, set<NoteBeat> > waveNoteBeats;
		for (list<Wave*>::iterator wave = waves.begin(); wave != waves.end(); wave++)
		{
			waveNoteBeats.insert(make_pair(*wave, (*wave)->getNoteBeats()));
		}

		for (int note = 0; note < PownConstants::NOTES; note++)
		{
			for (int beat = 0; beat < PownConstants::BEATS; beat++)
			{
				NoteBeat noteBeat(note, beat);
				int pos = getBrickPos(noteBeat);
				Brick* brick = bricks[pos];
				ofFloatColor color = beat == this->beat ? kBeatBoostColor : kDefaultBrickColor;

				for (map<Wave*, set<NoteBeat> >::const_iterator wnb = waveNoteBeats.begin(); wnb != waveNoteBeats.end(); wnb++)
					if (wnb->second.find(noteBeat) != wnb->second.end())
						color += wnb->first->getColor(); 

				color = light.getColor(color, brick->getCenter(), brick->getNormal());

				brick->setColor(color);
			}
		}

		for (int i = 0; i < bricks.size(); i++)
			bricks[i]->draw();
	}

	void BrickManager::updateBox(Box* box) const
	{
		for (vector<Brick*>::const_iterator brick = bricks.begin(); brick != bricks.end(); brick++)
		{
			if (box->testHit(*brick))
			{
				box->setNoteBeat((*brick)->getNoteBeat());
				break;
			}
		}
	}

}
