#include "Frustum.h"
#include "Constants.h"
#include "Globals.h"
#include "pointUtils.h"
#include "transformationUtils.h"
#include <pcl/common/transforms.h>
#include "ofGraphicsUtils.h"

namespace mapinect
{
	Frustum*	Frustum::instance = NULL;

	Frustum::Frustum()
	{
	}
	
	void Frustum::RecalculateFrustum()
	{
		if (instance == NULL)
		{
			instance = new Frustum();
		}

		// p es la posicion de la camara, en coordenadas de mundo
		ofVec3f p =  ofVec3f(gTransformation->getKinectEyeCoordinates());

		// direction es la dirección hacia la cuál está mirando el Kinect. Se debe normalizar.
		ofVec3f direction(gTransformation->getKinectDirectionCoordinates());
		direction = direction - p;
		direction.normalize();

		// up es en coordenadas de mundo, normalizado
		Eigen::Vector3f upEigen (0.0, -1.0, 0.0);		// Posicion inicial, en Sist. de Coord Local del Kinect
		upEigen = gTransformation->getWorldTransformation() * upEigen;
		ofVec3f up = ofVec3f(upEigen.x(), upEigen.y(), upEigen.z());	
		up = up - p;
		up.normalize();

		ofVec3f rightInit = ofVec3f(0,0,1).crossed(ofVec3f(0,1,0));
		// right es el producto cross entre direction y up
		ofVec3f right = direction.crossed(up);

		// fc es el punto intersección entre direction y el plano far
		ofVec3f fc = p + direction * Constants::CLOUD_Z_MAX;

		// nc es el punto intersección entre direction y el plano near
		ofVec3f nc = p + direction * Constants::NDISTANCE;

		// La primer letra es si es en el plano far o near
		// La segunda letra es si es top o bottom
		// La tercer letra es si es left o right
		instance->ftl = fc + (up * Constants::HFAR_2) - (right * Constants::WFAR_2);
		instance->sftl = getScreenCoords(instance->ftl);
//		instance->sftl.z = 0;
		instance->ftr = fc + (up * Constants::HFAR_2) + (right * Constants::WFAR_2);
		instance->sftr = getScreenCoords(instance->ftr); 
//		instance->sftr.z = 0;
		instance->fbl = fc - (up * Constants::HFAR_2) - (right * Constants::WFAR_2);
		instance->sfbl = getScreenCoords(instance->fbl);  
//		instance->sfbl.z = 0;
		instance->fbr = fc - (up * Constants::HFAR_2) + (right * Constants::WFAR_2);
		instance->sfbr = getScreenCoords(instance->fbr);
//		instance->sfbr.z = 0;

		instance->ntl = nc + (up * Constants::HNEAR_2) - (right * Constants::WNEAR_2);
		instance->sntl = getScreenCoords(instance->ntl);  
//		instance->sntl.z = 0; 
		instance->ntr = nc + (up * Constants::HNEAR_2) + (right * Constants::WNEAR_2);
		instance->sntr = getScreenCoords(instance->ntr);  
//		instance->sntr.z = 0;
		instance->nbl = nc - (up * Constants::HNEAR_2) - (right * Constants::WNEAR_2);
		instance->snbl = getScreenCoords(instance->nbl);  
//		instance->snbl.z = 0;
		instance->nbr = nc - (up * Constants::HNEAR_2) + (right * Constants::WNEAR_2);
		instance->snbr = getScreenCoords(instance->nbr);
//		instance->snbr.z = 0;

		// Crear los planos que limitan al cono de visión
		//	Pasar los vértices siempre en el mismo orden, sentido horario, visto desde afuera del cono, para que todas las normales apunten adentro	
		//	No se va a controlar para los planos near y far
		//	La nube en el getCloud ya se limita al CLOUD_MAX_Z, q es lo q estamos tomando como far
		instance->coneBottom = Plane3D(instance->nbl,instance->nbr,instance->fbr);
		instance->coneRightSide = Plane3D(instance->fbr,instance->nbr,instance->ntr);
		instance->coneTop = Plane3D(instance->ftl,instance->ftr,instance->ntr);
		instance->coneLeftSide = Plane3D(instance->ftl,instance->ntl,instance->nbl);
	}

	bool Frustum::IsInFrustum(const ofVec3f& vec)
	{
		if (instance == NULL)
		{
			instance = new Frustum();
			instance->RecalculateFrustum();
		}

		float distConeBottom = instance->coneBottom.signedDistance(vec);
		float distRightSide = instance->coneRightSide.signedDistance(vec);
		float distConeTop = instance->coneTop.signedDistance(vec);
		float distConeLeftSide = instance->coneLeftSide.signedDistance(vec);

		bool vecInsideConeBottom = instance->coneBottom.signedDistance(vec) < 0;
		bool vecInsideConeRightSide = instance->coneRightSide.signedDistance(vec) < 0;
		bool vecInsideConeTop = instance->coneTop.signedDistance(vec) < 0;
		bool vecInsideConeLeftSide = instance->coneLeftSide.signedDistance(vec) < 0;

		bool inViewField = vecInsideConeBottom
			   && vecInsideConeTop
			   && vecInsideConeLeftSide
			   && vecInsideConeRightSide;

//		if (!inViewField)
//			cout << "objecto fuera del frustum" << endl; 
//		else 
//			cout << "objeto dentro del cono de vision" << endl;

		return inViewField;
	}

	bool Frustum::IsInFrustum(const vector<ofVec3f>& vertices)
	{
		bool verticesInFrustum = true;
		for (int i = 0; i < vertices.size(); i++) {
			cout << vertices[i].x << ", " << vertices[i].y << ", " << vertices[i].z << "  -  " << IsInFrustum(vertices.at(i)) << endl;
			verticesInFrustum = verticesInFrustum && IsInFrustum(vertices.at(i));
		}
		
		if (verticesInFrustum)
			cout << "objecto dentro del frustum" << endl; 
		else 
			cout << "objeto fuera" << endl;

		return verticesInFrustum;
	}

	void Frustum::drawFrustum() 
	{
		if (instance != NULL)
		{	
			ofSetColor(255,255,255,160);
			ofDrawQuadTextured(instance->ftl, instance->ntl, instance->nbl, instance->fbl);
			ofDrawQuadTextured(instance->fbl, instance->fbr, instance->nbr, instance->nbl);
			ofDrawQuadTextured(instance->ftr, instance->ntr, instance->nbr, instance->fbr);
			ofDrawQuadTextured(instance->ftl, instance->ftr, instance->ntr, instance->ntl);

			ofLine(instance->ftl, instance->ntl);
			ofLine(instance->ftl, instance->fbl);
			ofLine(instance->ftr, instance->ntr);				
			ofLine(instance->ftr, instance->fbr);				
			ofLine(instance->fbl, instance->nbl);
			ofLine(instance->ntl, instance->nbl);
			ofLine(instance->fbr, instance->nbr);
			ofLine(instance->ntr, instance->nbr);

			ofLine(instance->nbl, instance->nbr);
			ofLine(instance->ntl, instance->ntr);

			ofLine(instance->fbl, instance->fbr);
			ofLine(instance->ftl, instance->ftr);
		}	

	}

	void Frustum::debugDrawFrustum()
	{
		if (instance != NULL)
		{	
			ofSetColor(255,255,255);
			ofLine(instance->sftl, instance->sntl);
			ofLine(instance->sftl, instance->sfbl);
			ofLine(instance->sftr, instance->sntr);				
			ofLine(instance->sftr, instance->sfbr);				
			ofLine(instance->sfbl, instance->snbl);
			ofLine(instance->sntl, instance->snbl);
			ofLine(instance->sfbr, instance->snbr);
			ofLine(instance->sntr, instance->snbr);

			ofLine(instance->snbl, instance->snbr);
			ofLine(instance->sntl, instance->sntr);

			ofLine(instance->sfbl, instance->sfbr);
			ofLine(instance->sftl, instance->sftr);
		}	
	}

}