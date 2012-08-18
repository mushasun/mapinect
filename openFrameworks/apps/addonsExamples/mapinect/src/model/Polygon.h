#ifndef MAPINECT_POLYGON_H__
#define MAPINECT_POLYGON_H__

#include "ModelObject.h"
#include "IPolygon.h"

#include "Polygon3D.h"

namespace mapinect {

	class Polygon;

	typedef boost::shared_ptr<Polygon> PolygonPtr;

	class Polygon : public ModelObject, public IPolygon
	{
		public:
			Polygon(const pcl::ModelCoefficients& coefficients)
				: name(kPolygonNameUnknown), mathModel()
					{ mathModel.setPlane(coefficients); container.reset(); }
			Polygon(const Polygon&);
			virtual ~Polygon()										{ }

			// IPolygon
			inline int						getId() const			{ return ModelObject::getId(); }
			inline const Polygon3D&			getMathModel() const	{ return mathModel; }
			inline const IObjectPtr&		getContainer() const	{ return container; }
			inline const IPolygonName&		getName() const			{ return name; }
						 int				getBestOriginVertexIndex() const;

			// Polygon specific method
			IPolygonPtr			clone() const;
			inline void			setContainer(const IObjectPtr& object)		{ container = object; }
			void				setVertex(int vertexNum, const ofVec3f& v);
			void				setVertexs(const vector<ofVec3f>& v);
			void				setPlane(const Plane3D& p);
			void				setVertexsOrdered(const vector<ofVec3f>& v);
			inline void			setName(const IPolygonName& newName)		{ name = newName; }
			inline Polygon3D&	getMathModel() 	{ return mathModel; }
			virtual void		draw();
		
		private:
			IPolygonName		name;
			IObjectPtr			container;
			Polygon3D			mathModel;
	};
}

#endif	// MAPINECT_POLYGON_H__