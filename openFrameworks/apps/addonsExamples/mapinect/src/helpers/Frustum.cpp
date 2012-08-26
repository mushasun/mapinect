#include "Frustum.h"
#include "Constants.h"
#include "Globals.h"
#include <pcl/common/transforms.h>

namespace mapinect
{
	Frustum*	Frustum::instance = NULL;

	Frustum::Frustum()
	{
		;
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

		// Se definen los 8 puntos del cono de visión
		ofVec3f ftl, ftr, fbl, fbr, ntl, ntr, nbl, nbr;
		// La primer letra es si es en el plano far o near
		// La segunda letra es si es top o bottom
		// La tercer letra es si es left o right
		ftl = fc + (up * Constants::HFAR_2) - (right * Constants::WFAR_2);
		ftr = fc + (up * Constants::HFAR_2) + (right * Constants::WFAR_2);
		fbl = fc - (up * Constants::HFAR_2) - (right * Constants::WFAR_2);
		fbr = fc - (up * Constants::HFAR_2) + (right * Constants::WFAR_2);

		ntl = nc + (up * Constants::HNEAR_2) - (right * Constants::WNEAR_2);
		ntr = nc + (up * Constants::HNEAR_2) + (right * Constants::WNEAR_2);
		nbl = nc - (up * Constants::HNEAR_2) - (right * Constants::WNEAR_2);
		nbr = nc - (up * Constants::HNEAR_2) + (right * Constants::WNEAR_2);

		// Crear los planos que limitan al cono de visión
		//	Pasar los vértices siempre en el mismo orden, sentido horario, visto desde afuera del cono, para que todas las normales apunten adentro	
		//	No se va a controlar para los planos near y far
		//	La nube en el getCloud ya se limita al CLOUD_MAX_Z, q es lo q estamos tomando como far
		instance->coneBottom = Plane3D(nbl,nbr,fbr);
		instance->coneRightSide = Plane3D(fbr,nbr,ntr);
		instance->coneTop = Plane3D(ftl,ftr,ntr);
		instance->coneLeftSide = Plane3D(ftl,ntl,nbl);
	}

	bool Frustum::IsInFrustum(ofVec3f vec)
	{
		if (instance == NULL)
		{
			instance = new Frustum();
			instance->RecalculateFrustum();
		}

		bool distConeBottom = instance->coneBottom.signedDistance(vec);
		bool distRightSide = instance->coneRightSide.signedDistance(vec);
		bool distConeTop = instance->coneTop.signedDistance(vec);
		bool distConeLeftSide = instance->coneLeftSide.signedDistance(vec);

		bool vecInsideConeBottom = instance->coneBottom.signedDistance(vec) < -Constants::FOV_MIN_DIST_CENTROID;
		bool vecInsideConeRightSide = instance->coneRightSide.signedDistance(vec) < -Constants::FOV_MIN_DIST_CENTROID;
		bool vecInsideConeTop = instance->coneTop.signedDistance(vec) < -Constants::FOV_MIN_DIST_CENTROID;
		bool vecInsideConeLeftSide = instance->coneLeftSide.signedDistance(vec) < -Constants::FOV_MIN_DIST_CENTROID;

		bool inViewField = vecInsideConeBottom
			   && vecInsideConeTop
			   && vecInsideConeLeftSide
			   && vecInsideConeRightSide;

		return inViewField;
	}

}