#ifndef MAPINECT_DATAOBJECT_H__
#define MAPINECT_DATAOBJECT_H__

#include "IObject.h"

#include "IPolygon.h"

namespace mapinect
{
	class DataObject : public IObject
	{
	public:
		DataObject(int id, const ofVec3f& center, const ofVec3f& scale,
			const ofVec3f& rotation, const vector<IPolygonPtr>& polygons)
			: id(id), center(center), scale(scale), rotation(rotation), polygons(polygons) { }
		virtual ~DataObject() { }

		inline	int							getId() const							{ return id; }

		inline	const ofVec3f&				getCenter() const						{ return center; }
		inline	const ofVec3f&				getScale() const						{ return scale; }
		inline	const ofVec3f&				getRotation() const						{ return rotation; }

				const IPolygonPtr&			getPolygon(const IPolygonName& name) const; 
		inline	const vector<IPolygonPtr>&	getPolygons() const						{ return polygons; }

	private:
		int						id;
		ofVec3f					center, scale, rotation;
		vector<IPolygonPtr>		polygons;

	};
}
#endif	// MAPINECT_DATAOBJECT_H__
