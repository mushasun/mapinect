#ifndef MAPINECT_CONSTANTS_H__
#define MAPINECT_CONSTANTS_H__

#define KINECT_DEFAULT_WIDTH		640
#define KINECT_DEFAULT_HEIGHT		480

namespace mapinect
{
	class Constants
	{
	public:
		static int					CLOUD_STRIDE_H;
		static int					CLOUD_STRIDE_V;

		static int					CLOUD_POINTS_MAX;
		static float				CLOUD_DENSITY;
		static float				CLOUD_VOXEL_SIZE;
		static float				CLOUD_Z_MAX;

		static int					CLOUD_POINTS;

	private:
		static float				TABLE_CLUSTER_MIN_PERCENT;
		static float				TABLE_CLUSTER_TOLERANCE_FACTOR;
		static float				TABLE_HEIGHT_TOLERANCE_FACTOR;

		static float				OBJECT_CLUSTER_MIN_PERCENT;
		static float				OBJECT_CLUSTER_TOLERANCE_FACTOR;
		static float				OBJECT_PLANE_TOLERANCE_FACTOR;

	public:
		static int					OBJECT_FRAMES_TO_ACCEPT;
		static int					OBJECT_INVALID_FRAMES_TO_RESET;
		static int					OBJECT_INVALID_FRAMES_TO_DELETE;
		static int					OBJECT_VOLUME_TOLERANCE;
		static int					OBJECT_LOD_MAX;
	private:
		static float				OBJECT_VERTEX_UNIFYING_DISTANCE_FACTOR;
	public:
		static float				OBJECT_CLOUD_DIFF_PERCENT;
	private:
		static float				OBJECT_RECALCULATE_TOLERANCE_FACTOR;
		static float				OBJECT_TRANSLATION_TOLERANCE_FACTOR;

	public:
		static int					TOUCH_FRAMES_TO_DISCARD;
	private:
		static float				TOUCH_DISTANCE_FACTOR;
		static float				TOUCH_TOLERANCE_FACTOR;
		static float				TOUCH_CLUSTER_MIN_PERCENT;
		static float				TOUCH_TRANSLATION_TOLERANCE_FACTOR;
	public:
		static int					TOUCH_MAX_PER_FACE;

		static void					LoadConstants();

		static int					CLOUD_STRIDE();
		static float				CLOUD_VOXEL_SIZE_FOR_STRIDE(int stride);
			
		static int					TABLE_CLUSTER_MIN_SIZE();
		static float				TABLE_CLUSTER_TOLERANCE();
		static float				TABLE_HEIGHT_TOLERANCE();

		static int					OBJECT_CLUSTER_MIN_SIZE();
		static float				OBJECT_CLUSTER_TOLERANCE();
		static float				OBJECT_PLANE_TOLERANCE();

		static float				OBJECT_VERTEX_UNIFYING_DISTANCE();
		static float				OBJECT_RECALCULATE_TOLERANCE();
		static float				OBJECT_TRANSLATION_TOLERANCE();

		static float				TOUCH_DISTANCE();
		static float				TOUCH_TOLERANCE();
		static int					TOUCH_CLUSTER_MIN_SIZE();
		static float				TOUCH_TRANSLATION_TOLERANCE();

		//contsantes de view field
		static float				NDISTANCE;
		static float				FOV_HORIZONTAL;
		static float				FOV_VERTICAL;
		static float				WFAR_2;
		static float				HFAR_2;
		static float				WNEAR_2;
		static float				HNEAR_2;

		// Para la calibración inicial de la mesa
		static int					TABLE_VERTEX_ESTIMATED_PIXEL_TOLERANCE;
		static float				TABLE_LENGTH_AB;
		static float				TABLE_LENGTH_AD;
	
	};
}

#endif	// MAPINECT_CONSTANTS_H__
