#include "ArmCalibration.h"

#include <pcl/common/transforms.h>

#include "Constants.h"
#include "Feature.h"
#include "Globals.h"
#include "ofxKinect.h"
#include "ofxXmlSettings.h"
#include "pointUtils.h"
#include "VM.h"

namespace armCalibration {

	//--------------------------------------------------------------
	void ArmCalibration::setup()
	{
		ofSetWindowTitle("arm calibration");

		LoadFeatures();

		gKinect = new ofxKinect();
		if (IsFeatureKinectActive())
		{
			KINECT_WIDTH = KINECT_DEFAULT_WIDTH;
			KINECT_HEIGHT = KINECT_DEFAULT_HEIGHT;
			MAX_Z = 4.0f;

			double fx_d, fy_d, fx_rgb, fy_rgb; 
			float  cx_d, cy_d, cx_rgb, cy_rgb;
			ofVec3f T_rgb;
			ofMatrix4x4 R_rgb;
			getKinectCalibData("data/calib/kinect-calib-27May.yml",
						fx_d, fy_d, cx_d, cy_d,
						fx_rgb, fy_rgb, cx_rgb, cy_rgb,
						T_rgb, R_rgb);
			gKinect->getCalibration().setCalibValues(
						fx_d, fy_d, cx_d, cy_d,
						fx_rgb, fy_rgb, cx_rgb, cy_rgb,
						T_rgb, R_rgb);

			gKinect->init();
			gKinect->setVerbose(true);
			gKinect->open();
		}

		arduino.setup();

	}

	//--------------------------------------------------------------
	void ArmCalibration::exit()
	{
	}

	//--------------------------------------------------------------
	void ArmCalibration::draw()
	{
		if (IsFeatureKinectActive())
		{
			ofSetHexColor(0xFFFFFF);
			gKinect->drawDepth(20, 20, KINECT_DEFAULT_WIDTH, KINECT_DEFAULT_HEIGHT);
		}

		ofSetHexColor(0x000000);
		stringstream reportStream;
		reportStream
			<< "   ARM_LENGTH: " << Arduino::ARM_LENGTH << endl
			<< "KINECT_HEIGHT: " << Arduino::KINECT_HEIGHT << endl
			<< "MOTORS_HEIGHT: " << Arduino::MOTORS_HEIGHT << endl;
		ofDrawBitmapString(reportStream.str(), 20, KINECT_DEFAULT_HEIGHT + 40);
	}

	//--------------------------------------------------------------
	void ArmCalibration::debugDraw()
	{
	}

	//--------------------------------------------------------------
	void ArmCalibration::update()
	{
		if (IsFeatureKinectActive())
		{
			gKinect->update();
		}
	}

	//--------------------------------------------------------------
	void ArmCalibration::keyPressed(int key)
	{
		enum ArmCalibrationMode
		{
			kReadingClouds = 0,
			kCheckingValues
		};
		static ArmCalibrationMode mode = kReadingClouds;

		if (key == 'm')
		{
			mode = mode == kReadingClouds ? kCheckingValues : kReadingClouds;
		}

		if (mode == kReadingClouds)
		{
			switch (key)
			{
				case '1':
					arduino.setArm3dCoordinates(ofVec3f(Arduino::ARM_LENGTH, 0, 0.15));
					arduino.lookAt(ofVec3f(0.35, -0.16, 0.10));
					break;
				case '2':

					break;
				case '3':

					break;
				case ' ':
					storeCloud();
					break;
				case 'k':
					saveRawClouds();
					break;
				case 'l':
					loadRawClouds();
					break;
			}
		}
		else
		{
			static short editingValue = 1;
			const float kValueChange = 0.002;
			switch (key)
			{
				case 'z':
					changeValue(Arduino::ARM_LENGTH, -kValueChange);
					break;
				case 'a':
					changeValue(Arduino::ARM_LENGTH, kValueChange);
					break;
				case 'x':
					changeValue(Arduino::KINECT_HEIGHT, -kValueChange);
					break;
				case 's':
					changeValue(Arduino::KINECT_HEIGHT, kValueChange);
					break;
				case 'c':
					changeValue(Arduino::MOTORS_HEIGHT, -kValueChange);
					break;
				case 'd':
					changeValue(Arduino::MOTORS_HEIGHT, kValueChange);
					break;

				case ' ':
					saveTransformedClouds();
					break;
			}
		}
	}

	//--------------------------------------------------------------
	void ArmCalibration::changeValue(float& value, float delta)
	{
		value += delta;
	}

	//--------------------------------------------------------------
	void ArmCalibration::storeCloud()
	{
		PCPtr cloud = getCloud(4);
		storedClouds.push_back(MotorRotationsCloud(arduino.motorAngles(), cloud));
	}

#define STOREDCLOUD_CONFIG	"STOREDCLOUD:"

	enum FileType
	{
		kFileTypeXML = 0,
		kFileTypePCD
	};

	string getRawFilename(int i, FileType type)
	{
		string filename("storedCloud" + ofToString(i));
		if (type == kFileTypeXML)
			filename += ".xml";
		else if (type == kFileTypePCD)
			filename += ".pcd";
		return filename;
	}

	string getTransformedFilename(int i)
	{
		return "transformedCloud" + ofToString(i) + ".pcd";
	}

	//--------------------------------------------------------------
	void ArmCalibration::saveRawClouds()
	{
		int i = 1;
		for (vector<MotorRotationsCloud>::const_iterator mrc = storedClouds.begin(); mrc != storedClouds.end(); ++mrc)
		{
			ofxXmlSettings XML;
			XML.setValue(STOREDCLOUD_CONFIG "R1", mrc->rotations[0]);
			XML.setValue(STOREDCLOUD_CONFIG "R2", mrc->rotations[1]);
			XML.setValue(STOREDCLOUD_CONFIG "R4", mrc->rotations[2]);
			XML.setValue(STOREDCLOUD_CONFIG "R8", mrc->rotations[3]);
			XML.saveFile(getRawFilename(i, kFileTypeXML));
			saveCloudAsFile(getRawFilename(i, kFileTypePCD), *mrc->cloud);
			i++;
		}
	}

	//--------------------------------------------------------------
	void ArmCalibration::loadRawClouds()
	{
		ofxXmlSettings XML;
		int i = 1;
		while (XML.loadFile(getRawFilename(i, kFileTypeXML)))
		{
			signed int* rotations = new signed int[4];
			rotations[0] = XML.getValue(STOREDCLOUD_CONFIG "R1", 0);
			rotations[1] = XML.getValue(STOREDCLOUD_CONFIG "R2", 0);
			rotations[2] = XML.getValue(STOREDCLOUD_CONFIG "R4", 0);
			rotations[3] = XML.getValue(STOREDCLOUD_CONFIG "R8", 0);
			PCPtr cloud = loadCloud(getRawFilename(i, kFileTypePCD));
			storedClouds.push_back(MotorRotationsCloud(rotations, cloud));
			i++;
		}
	}

	//--------------------------------------------------------------
	void ArmCalibration::saveTransformedClouds()
	{
		int i = 1;
		for (vector<MotorRotationsCloud>::const_iterator mrc = storedClouds.begin(); mrc != storedClouds.end(); ++mrc)
		{
			Eigen::Affine3f transformationMatrix =
				arduino.getWorldTransformation(mrc->rotations[0], mrc->rotations[1], mrc->rotations[2], mrc->rotations[3]);
			PC transformedCloud;
			pcl::transformPointCloud(*mrc->cloud, transformedCloud, transformationMatrix);
			saveCloudAsFile(getTransformedFilename(i), transformedCloud);
			i++;
		}

		string cmd = "pcd_viewer_release.exe";
		for (int j = 1; j < i; j++)
			cmd += " " + getTransformedFilename(j);
		system(cmd.c_str());
	}

	//--------------------------------------------------------------
	void ArmCalibration::keyReleased(int key)
	{
	}

	//--------------------------------------------------------------
	void ArmCalibration::windowMoved(int x, int y)
	{
	}

	//--------------------------------------------------------------
	void ArmCalibration::mouseMoved(int x, int y)
	{
	}

	//--------------------------------------------------------------
	void ArmCalibration::mouseDragged(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void ArmCalibration::mousePressed(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void ArmCalibration::mouseReleased(int x, int y, int button)
	{
	}

	//--------------------------------------------------------------
	void ArmCalibration::dragEvent(ofDragInfo info)
	{
	}

}
