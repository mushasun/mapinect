#include "vm.h"

#include "Feature.h"
#include "ofxXmlSettings.h"
#include "ofVec3f.h"
#include "Globals.h"

namespace mapinect {

#define		VM_CONFIG			"VMConfig:"

	static float transX =   0;
	static float transY =   0;
	static float transZ =   0;		
	static float rotYAxis = 0;

	static ofVec3f currentModelviewT(0,0,0);
	static ofVec3f modelviewRight(0,0,0);
	static ofVec3f modelviewUp(0,0,0);
	static ofVec3f modelviewFwd(0,0,0);

	static float proj_matrix_glproj[16];
	static float proj_fx;
	static float proj_fy;
	static float proj_cx;
	static float proj_cy;

	static float proj_matrix_RT[16];
	static float inv_proj_RT_premult[16];

	static float kinect_matrix_RT[16];
	static float inv_kinect_RT_premult[16];

	static ofVec3f cameraPosition(0,0,0);
	static ofVec3f cameraLookAt(0,0,0);

	static ofVec3f proj_loc, proj_fwd, proj_up, proj_trg;

	static bool bCustomFullscreen = false;

	std::string VM::projCalibFile = "";
	std::string VM::kinectCalibFile = "";

	float VM::projWidth = 0;
	float VM::projHeight = 0;

	float VM::nearPlane = 0;
	float VM::farPlane = 0;


	//--------------------------------------------------------------
	void VM::setup() {
		CHECK_ACTIVE;

		ofxXmlSettings XML;
		if(XML.loadFile("VM_Config.xml")) {

			nearPlane = XML.getValue(VM_CONFIG "NEAR_PLANE", 0.1);
			farPlane = XML.getValue(VM_CONFIG "FAR_PLANE", 20.0);

			VM::projCalibFile = XML.getValue(VM_CONFIG "PROJ_CALIB", "data/calib/projector_calibration.yml");
			VM::kinectCalibFile = XML.getValue(VM_CONFIG "KINECT_CALIB", "data/calib/kinect_calibration.yml");

			projWidth = XML.getValue(VM_CONFIG "PROJ_WIDTH", 1280);
			projHeight = XML.getValue(VM_CONFIG "PROJ_HEIGHT", 768);
		}

 		loadProjCalibData(const_cast<char*>(projCalibFile.c_str()));
		loadKinectExtrinsics(const_cast<char*>(kinectCalibFile.c_str()));		
//		keyPressed('0');

		glEnable(GL_DEPTH_TEST);
	}

	// Load calibration parameters for projector
	//		Camara Lucida, www.camara-lucida.com.ar
	void VM::loadProjCalibData(char* projCalibFile) {
		
		CHECK_ACTIVE;

		// Load matrices from Projector's calibration file		
		CvMat* projector_intrinsics = (CvMat*) cvLoad(projCalibFile, NULL, "proj_intrinsics");
		CvMat* projector_size = (CvMat*) cvLoad(projCalibFile, NULL, "proj_size");
		CvMat* projector_R = (CvMat*) cvLoad(projCalibFile, NULL, "R");
		CvMat* projector_T = (CvMat*) cvLoad(projCalibFile, NULL, "T");

		// Obtain projector intrinsic parameters
		proj_fx = (float) cvGetReal2D(projector_intrinsics, 0, 0);
		proj_fy = (float) cvGetReal2D(projector_intrinsics, 1, 1);
		proj_cx = (float) cvGetReal2D(projector_intrinsics, 0, 2);
		proj_cy = (float) cvGetReal2D(projector_intrinsics, 1, 2);

		float proj_width = (float) cvGetReal2D(projector_size, 0, 0); 
		float proj_height = (float) cvGetReal2D(projector_size, 0, 1);
		if (proj_width != projWidth || proj_height != projHeight)
			printf("Projector Width and Height from VM_CONFIG and Calib file don't match\n");
				
		// Create projection matrix for projector
		float A = 2. * proj_fx  / projWidth;
		float B = 2. * proj_fy / projHeight;
		float C = 2. * (proj_cx / projWidth) - 1.;
		float D = 2. * (proj_cy / projHeight) - 1.;
		float E = - (farPlane + nearPlane) / (farPlane - nearPlane);
		float F = -2. * farPlane * nearPlane / (farPlane - nearPlane);				
		proj_matrix_glproj[0]= A;	proj_matrix_glproj[4]= 0;	proj_matrix_glproj[8] =  C;	 proj_matrix_glproj[12]= 0;
		proj_matrix_glproj[1]= 0;	proj_matrix_glproj[5]= B;	proj_matrix_glproj[9] =  D;	 proj_matrix_glproj[13]= 0;		
		proj_matrix_glproj[2]= 0;	proj_matrix_glproj[6]= 0;	proj_matrix_glproj[10]=  E;	 proj_matrix_glproj[14]= F;
		proj_matrix_glproj[3]= 0;	proj_matrix_glproj[7]= 0;	proj_matrix_glproj[11]= -1;	 proj_matrix_glproj[15]= 0;

		float* projMatrixRT = getTransformationMatrixOpenGL(projector_R, projector_T);
		if (projMatrixRT != NULL) {
			for(int i = 0; i < 16; i++) {
				proj_matrix_RT[i] = projMatrixRT[i]; 
			}
		}		

		float* inverseProjMatrixRT = getInverseTransformationMatrixOpenGL(projector_R, projector_T);
		if (inverseProjMatrixRT != NULL) {
			for(int i = 0; i < 16; i++) {
				inv_proj_RT_premult[i] = inverseProjMatrixRT[i]; 
			}
		}		

		// Release matrices loaded with Projector's calib values 
		cvReleaseMat(&projector_intrinsics);
		cvReleaseMat(&projector_size);
		cvReleaseMat(&projector_R);
		cvReleaseMat(&projector_T); 
	}

	// Load kinect extrinsic parameters (matrices R and T)
	void VM::loadKinectExtrinsics(char* kinect_calib_file) {
		
		CHECK_ACTIVE;

		// Load matrices R and T from Kinect's calibration file		
		CvMat* kinect_R = (CvMat*) cvLoad(kinect_calib_file, NULL, "R");
		CvMat* kinect_T = (CvMat*) cvLoad(kinect_calib_file, NULL, "T");
		
		float* kinectMatrixRT = getTransformationMatrixOpenGL(kinect_R, kinect_T);
		if (kinectMatrixRT != NULL) {
			for(int i = 0; i < 16; i++) {
				kinect_matrix_RT[i] = kinectMatrixRT[i]; 
			}
		}		

		float* inverseKinectMatrixRT = getInverseTransformationMatrixOpenGL(kinect_R, kinect_T);
		if (inverseKinectMatrixRT != NULL) {
			for(int i = 0; i < 16; i++) {
				inv_kinect_RT_premult[i] = inverseKinectMatrixRT[i]; 
			}
		}		

		// Release matrices loaded with Kinect's calib values 
		cvReleaseMat(&kinect_R);
		cvReleaseMat(&kinect_T); 
	}

	//--------------------------------------------------------------
	void VM::setupView() 
	{
		CHECK_ACTIVE;

		glViewport(0, 0, projWidth, projHeight);

		glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			glMultMatrixf(proj_matrix_glproj); 

/*		TRANSFORMACIONES ENTRE SISTEMAS DE COORDENADAS
		* Sist. de Coord. de OpenCV (RGBDemo)
			- Origen: cámara RGB / receptor IR
			- +X hacia izquierda (mirando de frente al Kinect)
			- +Y hacia abajo
			- +Z hacia donde uno está parado (mirando de frente al Kinect)
		* Sist. de Coord. de ofxKinect y de nuestro modelo
			- +X, +Y, +Z idem al Sist. Coord. de OpenCV (RGBDemo)
		* Sist. de Coord. de OpenGL
			- +X hacia izquierda (mirando de frente al Kinect)
			- +Y hacia arriba
			- +Z hacia el Kinect (mirando de frente al Kinect)
		=> Pasar del sist. de coord. de profundidad a RGB:
			Aplicar (R|T) del archivo de calibración del Kinect
		=> Pasar del sist. de coord. de RGB a proyector:
			Aplicar transformación inversa del (R|T) del archivo de calibración del proyector
			inv(R|T) = (T*R)^-1 = R^-1 * T^-1 = -R^-1*T
*/

		glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();
			glTranslatef(transX,transY,transZ);
			//		glRotatef(rotYAxis,0,1,0);
			glScalef(1,-1,-1);											//3 - Pasar del sist. de coord. de nuestro modelo al de OpenGL (X,-Y,-Z)
			glMultMatrixf(inv_proj_RT_premult);							//2 - Pasar de rgb-camera space al sistema de coord. del proyector
			glMultMatrixf(kinect_matrix_RT);							//1 - Pasar de depth-camera space a rgb-camera space	
			Eigen::Affine3f inverseWorldTransformationMatrix = gTransformation->getInverseWorldTransformation();
			glMultMatrixf(inverseWorldTransformationMatrix.data()); //0 - Aplicar transformación de mundo real

			// Check current translation after multiplying matrices
			GLfloat matrix[16]; 
			glGetFloatv (GL_MODELVIEW_MATRIX, matrix);
			currentModelviewT.x = matrix[12];
			currentModelviewT.y = matrix[13];
			currentModelviewT.z = matrix[14];
			modelviewRight.x = matrix[0];
			modelviewRight.y = matrix[1];
			modelviewRight.z = matrix[2];
			modelviewUp.x = matrix[4];
			modelviewUp.y = matrix[5];
			modelviewUp.z = matrix[6];
			modelviewFwd.x = -matrix[8];
			modelviewFwd.y = -matrix[9];
			modelviewFwd.z = -matrix[10];

	}

	//--------------------------------------------------------------
	void VM::draw() {
		CHECK_ACTIVE;
	}

	//--------------------------------------------------------------
	void VM::endView() {
		CHECK_ACTIVE;
		
		//ofPopMatrix();
	}

	//--------------------------------------------------------------
	void VM::update() {
		CHECK_ACTIVE;
	}

	//--------------------------------------------------------------
	void VM::keyPressed(int key) {
		CHECK_ACTIVE;
				
		float varProj = 2.0f;
		float varTrans = 0.01f;

		switch (key) {
			/*********************
				fy		++	= UP
				fy		--	= DOWN
				fx		++	= RIGHT
				fx		--	= LEFT
				transY	++	= w
				transY	--	= s
				transX	++	= d
				transX	--	= a
				transZ	++	= +
				transZ	--	= -
			
			********************/	
			case 't':
				proj_fy +=varProj;
				printf("proj_fy increased: %f \n",proj_fy);
				setProjMatrix(proj_fx, proj_fy, proj_cx, proj_cy);
				break;
			case 'g':
				proj_fy -=varProj;
				printf("proj_fy decreased: %f \n",proj_fy);
				setProjMatrix(proj_fx, proj_fy, proj_cx, proj_cy);
				break;
			case 'f':
				proj_fx -=varProj;
				printf("proj_fx decreased: %f \n",proj_fx);
				setProjMatrix(proj_fx, proj_fy, proj_cx, proj_cy);
				break;
			case 'h':
				proj_fx +=varProj;
				printf("proj_fx increased: %f \n",proj_fx);
				setProjMatrix(proj_fx, proj_fy, proj_cx, proj_cy);
				break;
			case 'j':
				proj_cx -=varProj;
				printf("proj_cx decreased: %f \n",proj_cx);
				setProjMatrix(proj_fx, proj_fy, proj_cx, proj_cy);
				break;
			case 'l':
				proj_cx +=varProj;
				printf("proj_cx increased: %f \n",proj_cx);
				setProjMatrix(proj_fx, proj_fy, proj_cx, proj_cy);
				break;
			case 'i':
				proj_cy +=varProj;
				printf("proj_cy increased: %f \n",proj_cy);
				setProjMatrix(proj_fx, proj_fy, proj_cx, proj_cy);
				break;
			case 'k':
				proj_cy -=varProj;
				printf("proj_cy decreased: %f \n",proj_cy);
				setProjMatrix(proj_fx, proj_fy, proj_cx, proj_cy);
				break;
			case 'w':
				transY += varTrans;
				printf("Y translation inc: %f \n", transY);
				break;
			case 's':
				transY -= varTrans;
				printf("Y translation dec: %f \n", transY);
				break;
			case 'a':
				transX -= varTrans;
				printf("X translation dec: %f \n", transX);
				break;
			case 'd':
				transX += varTrans;
				printf("X translation inc: %f \n", transX);
				break;
			case '-':
				transZ -= varTrans;
				printf("Z translation dec: %f \n", transZ);
				break;
			case '+':
				transZ += varTrans;
				printf("Z translation inc: %f \n", transZ);
				break;
			case '1':
				loadCalibParams();
				printf("Parameters loaded from VM config file\n");
				keyPressed('q');
				break;
			case '2':
				saveCurrentCalibParams();
				printf("Parameters saved to VM config file\n");
				keyPressed('q');
				break;

			/*********************
			  SHOW STATUS	 - q
			*********************/
			case 'q':
				printf("Current parameters: \n");
				printf("	Modelview Trans [x,y,z] = (%.4f,%.4f,%.4f) \n",currentModelviewT.x, currentModelviewT.y, currentModelviewT.z);
				printf("	Proj intrinsics [fx,fy,cx,cy] = (%.1f,%.1f,%.1f,%.1f) \n", proj_fx, proj_fy, proj_cx, proj_cy);
				printf("	Model glTransl  [x,y,z] = (%.4f,%.4f,%.4f) \n", transX, transY, transZ);
				break;
		case 'o':
			rotYAxis -= 0.5;
			printf("decreased rotation angle Axis Y: %f \n",rotYAxis);
			break;
		case 'p':
			rotYAxis += 0.5;
			printf("increased rotation angle Axis Y: %f \n",rotYAxis);
			break;
		}
	}

	//--------------------------------------------------------------
	void VM::keyReleased(int key)
	{
		CHECK_ACTIVE;
	}

	//--------------------------------------------------------------
	void VM::windowMoved(int x, int y)
	{
		CHECK_ACTIVE;
	}

	//--------------------------------------------------------------
	void VM::mouseMoved(int x, int y)
	{
		CHECK_ACTIVE;
	}

	//--------------------------------------------------------------
	void VM::mouseDragged(int x, int y, int button)
	{
		CHECK_ACTIVE;
	}

	//--------------------------------------------------------------
	void VM::mousePressed(int x, int y, int button)
	{
		CHECK_ACTIVE;
	}

	//--------------------------------------------------------------
	void VM::mouseReleased(int x, int y, int button)
	{
		CHECK_ACTIVE;
	}

	//--------------------------------------------------------------
	void VM::dragEvent(ofDragInfo info)
	{
		CHECK_ACTIVE;
	}

	//--------------------------------------------------------------
	bool VM::isActive()
	{
		return IsFeatureVMActive();
	}

	void VM::setProjMatrix(float fx, float fy, float cx, float cy)
	{
		// Update projection matrix for projector
		//	A	0	C	0
		//	0	B	D	0
		//	0	0	E	F
		//	0	0	-1	0
		proj_matrix_glproj[0] = 2. * fx / projWidth;
		proj_matrix_glproj[5] = 2. * fy / projHeight;
		proj_matrix_glproj[8] = 2. * (cx / projWidth) - 1.;
		proj_matrix_glproj[9] = 2. * (cy / projHeight) - 1.;
	}

	std::string VM::getKinectCalibFile()
	{
		return kinectCalibFile;
	}

	float* VM::getTransformationMatrixOpenGL(CvMat* R, CvMat* T)
	{
		float* matrixRT = new float[16];		
		// Create homogeneous transformation matrix (R|T)
		//RT:
		//		r00	r01 r02	tx	
		//		r10	r11	r12	ty
		//		r20	r21	r22	tz
		//		0	0	0	1
		// In OpenGl, matrices use col-major order, so RT should be stored as: {xx, xy, xz, 0, yx, yy, yz, 0, zx, zy, zz, 0, tx, ty, tz, 1} 
		int j = 0;
		for(int i=0; i<3; i++){
			matrixRT[j] = (float) cvGetReal2D(R, 0, i);
			matrixRT[j+1] = (float) cvGetReal2D(R, 1, i);
			matrixRT[j+2] = (float) cvGetReal2D(R, 2, i);
			matrixRT[j+3] = 0;
			j+=4;
		}
		for(int i=0; i<3; i++){
			matrixRT[i+12] = (float) cvGetReal2D(T,i,0);
		}
		matrixRT[15] = 1;

		return matrixRT;
	}

	float* VM::getInverseTransformationMatrixOpenGL(CvMat* cvMatR, CvMat* cvMatT)
	{
		float* inverseMatrixRT = new float[16];		
		// Calculate inverse transformation
		//R:
		//	xx	xy	xz	0
		//	yx	yy	yz	0
		//	zx	zy	zz	0
		//	0	0	0	1
		//T:
		//	1	0	0	tx	
		//	0	1	0	ty
		//	0	0	1	tz
		//	0	0	0	1
		// (R|T) = T*R => Inv(T*R) = (T*R)^-1 = R^-1 * T^-1 = R^-1 * -T 
		//Inv(R|T):
		//		xx	yx	zx	-tx*xx -ty*yx -tz*zx	
		//		xy	yy	zy	-ty*xy -ty*yy -tz*zy
		//		xz	yz	zz	-tz*xz -ty*yz -tz*zz
		//		0	0	0	1
		// In OpenGl, matrices use col-major order, so RT should be stored as: {xx, yx, zx, 0, xy, yy, zy, 0, xz, yz, zz, 0, -tx, -ty, -tz, 1} 
		ofVec3f T((float) cvGetReal2D(cvMatT,0,0), (float) cvGetReal2D(cvMatT,1,0), (float) cvGetReal2D(cvMatT,2,0));
		ofMatrix4x4 R((float) cvGetReal2D(cvMatR,0,0), (float) cvGetReal2D(cvMatR,0,1), (float) cvGetReal2D(cvMatR,0,2),0,
		 (float) cvGetReal2D(cvMatR,1,0), (float) cvGetReal2D(cvMatR,1,1), (float) cvGetReal2D(cvMatR,1,2), 0,
		 (float) cvGetReal2D(cvMatR,2,0), (float) cvGetReal2D(cvMatR,2,1), (float) cvGetReal2D(cvMatR,2,2), 0,
		 0,0,0,1);		
		ofMatrix4x4 inverseRT = ofMatrix4x4(R);
		inverseRT.preMultTranslate(-T);
		//inverseRT = ofMatrix4x4::getTransposedOf(inverseRT);
		//inv_RT_premult = inverseRT.getPtr(); // Devuelve la matriz en row-major order
		// No la traspongo para poder sacar fácil al float* en orden por columnas, como necesita opengl  
		int j = 0;
		for(int i=0; i<4; i++){
			ofVec4f row_i = inverseRT.getRowAsVec4f(i);
			inverseMatrixRT[j]   = row_i.x;
			inverseMatrixRT[j+1] = row_i.y;
			inverseMatrixRT[j+2] = row_i.z;
			inverseMatrixRT[j+3] = row_i.w;
			j+=4;
		}

		return inverseMatrixRT;
	}

	//--------------------------------------------------------------
	// Get calibration parameters for Kinect's rgb and depth cameras
	//		Camara Lucida, www.camara-lucida.com.ar
	void getKinectCalibData(const string& kinect_calib_file,
							double& d_fx, double& d_fy, float& d_cx, float& d_cy,
							double& rgb_fx, double& rgb_fy, float& rgb_cx, float& rgb_cy,
							ofVec3f& T, ofMatrix4x4& R)
	{
		// Load matrices from Kinect's calibration file		
		CvMat* rgb_intrinsics = (CvMat*) cvLoad(kinect_calib_file.c_str(), NULL, "rgb_intrinsics");
		// CvMat* rgb_size = (CvMat*) cvLoad(kinect_calib_file, NULL_, "rgb_size");		// Size is 640x480 fixed for both RGB and IR
		CvMat* depth_intrinsics = (CvMat*) cvLoad(kinect_calib_file.c_str(), NULL, "depth_intrinsics");
		//CvMat* depth_size = (CvMat*) cvLoad(kinect_calib_file, NULL, "depth_size");	// Size is 640x480 fixed for both RGB and IR
		CvMat* kinect_R = (CvMat*) cvLoad(kinect_calib_file.c_str(), NULL, "R");
		CvMat* kinect_T = (CvMat*) cvLoad(kinect_calib_file.c_str(), NULL, "T");
			
		// Obtain RGB intrinsic parameters
		rgb_fx = (double) cvGetReal2D(rgb_intrinsics, 0, 0);
		rgb_fy = (double) cvGetReal2D(rgb_intrinsics, 1, 1);
		rgb_cx = (float) cvGetReal2D(rgb_intrinsics, 0, 2);
		rgb_cy = (float) cvGetReal2D(rgb_intrinsics, 1, 2);

		// Obtain Depth intrinsic parameters
		d_fx = (double) cvGetReal2D(depth_intrinsics, 0, 0);
		d_fy = (double) cvGetReal2D(depth_intrinsics, 1, 1);
		d_cx = (float) cvGetReal2D(depth_intrinsics, 0, 2);
		d_cy = (float) cvGetReal2D(depth_intrinsics, 1, 2);


		T.x = (float) cvGetReal2D(kinect_T,0,0);
		T.y = (float) cvGetReal2D(kinect_T,1,0);
		T.z = (float) cvGetReal2D(kinect_T,2,0);

		R.set((float)cvGetReal2D(kinect_R, 0, 0), (float)cvGetReal2D(kinect_R, 0, 1), (float)cvGetReal2D(kinect_R, 0, 2), 0,
			  (float)cvGetReal2D(kinect_R, 1, 0), (float)cvGetReal2D(kinect_R, 1, 1), (float)cvGetReal2D(kinect_R, 1, 2), 0,
			  (float)cvGetReal2D(kinect_R, 2, 0), (float)cvGetReal2D(kinect_R, 2, 1), (float)cvGetReal2D(kinect_R, 2, 2), 0,
			  0, 0, 0, 1);

		// Release matrices loaded with Kinect's calib values 
		cvReleaseMat(&rgb_intrinsics);
		cvReleaseMat(&depth_intrinsics);
		cvReleaseMat(&kinect_R);
		cvReleaseMat(&kinect_T); 
	}

	// Load calib parameters
	void VM::loadCalibParams() {
		ofxXmlSettings XML;
		if(XML.loadFile("VM_Config.xml")) {

			proj_fx = XML.getValue(VM_CONFIG "PROJ_FX", 1978.0);
			proj_fy = XML.getValue(VM_CONFIG "PROJ_FY", 1586.0);
			proj_cx = XML.getValue(VM_CONFIG "PROJ_CX", 642.0);
			proj_cy = XML.getValue(VM_CONFIG "PROJ_CY", 373.0);

			transX = XML.getValue(VM_CONFIG "TRANS_X", -0.112);
			transY = XML.getValue(VM_CONFIG "TRANS_Y",  0.020);
			transZ = XML.getValue(VM_CONFIG "TRANS_Z",  0.000);
		}
		 
		VM::setProjMatrix(proj_fx, proj_fy, proj_cx, proj_cy);

	}

	void VM::saveCurrentCalibParams() {
		ofxXmlSettings XML;
		if(XML.loadFile("VM_Config.xml")) {
			XML.setValue(VM_CONFIG "PROJ_FX", proj_fx);
			XML.setValue(VM_CONFIG "PROJ_FY", proj_fy);
			XML.setValue(VM_CONFIG "PROJ_CX", proj_cx);
			XML.setValue(VM_CONFIG "PROJ_CY", proj_cy);
			
			XML.setValue(VM_CONFIG "TRANS_X", transX);
			XML.setValue(VM_CONFIG "TRANS_Y", transY);
			XML.setValue(VM_CONFIG "TRANS_Z", transZ);

			XML.saveFile("VM_Config.xml");
		}
		 
	}

}
