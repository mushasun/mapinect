#include "Light.h"

#include "ofGraphicsUtils.h"
#include "ofVecUtils.h"

namespace pown
{
	Light::Light(const ofFloatColor& ambient, const ofFloatColor& diffuse, const ofVec3f& pos)
		: pos(pos)
	{
	}

	ofFloatColor Light::getColor(const ofFloatColor& color, const ofVec3f& vertex, const ofVec3f& normal) const
	{
		ofFloatColor result = kRGBBlack;
		ofVec3f d = (pos - vertex).normalized();
		float fdist = 1.0f / pos.distanceSquared(vertex);
		float fnorm = d.dot(normal);
		if (fnorm > 0)
		{
			fnorm = pow(fnorm, 4);
			result = color * diffuse * (fnorm * fdist);
		}
		result += color * ambient;
		return result;
	}

}
