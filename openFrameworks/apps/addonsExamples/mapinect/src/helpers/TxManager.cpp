#include "TxManager.h"

namespace mapinect {

	TxManager::TxManager(ofxFenster* f) {
		fenster = f;
	}

	GLuint TxManager::loadImageTexture(string imgFile)
	{
		GLuint result = -1;
		fenster->toContext();

		if (!imgFile.empty()) {
			ofImage img;
			if (img.loadImage(imgFile)) {
				unsigned char* imgPixels = img.getPixels();
				//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
				glEnable(GL_TEXTURE_2D);
				glPixelStorei (GL_UNPACK_ALIGNMENT, 1);
				glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
				//glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
  				glGenTextures(1, &result);
				glBindTexture(GL_TEXTURE_2D, result);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);	
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, img.getWidth(), img.getHeight(), 0, GL_RGB/*GL_RGBA*/,
					GL_UNSIGNED_BYTE, imgPixels);
				img.setUseTexture(true);
				glFlush();
			}
		}

		fenster->toMainContext();
		return result;
	} 

	GLuint TxManager::loadVideoTexture(string videoFile)
	{
		GLuint result = -1;
		fenster->toContext();

		if (!videoFile.empty()) {
			ofVideoPlayer* video = new ofVideoPlayer();
			video->loadMovie(videoFile);			
			if (!video->isLoaded()) {
				return result;			
			} else {
				// Start playing and get first image
				video->play(); 		
				unsigned char* videoPixels = video->getPixels();

				//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
				glEnable(GL_TEXTURE_2D);
				glPixelStorei (GL_UNPACK_ALIGNMENT, 1);
				glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
  				// Generate and bind texture
				glGenTextures(1, &result);
				glBindTexture(GL_TEXTURE_2D, result);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);	
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, video->getWidth(), video->getHeight(), 0, GL_RGB,
					GL_UNSIGNED_BYTE, videoPixels);				

				// Add to collection
				videoMap.insert(std::pair<GLuint,ofVideoPlayer*>(result, video));
				videoPix.insert(std::pair<GLuint,unsigned char*>(result,videoPixels));
			}

			fenster->toMainContext();
			return result;
		}
	}

	void TxManager::updateVideoTexture(GLuint textureId) {
		std::map<GLuint, ofVideoPlayer*>::const_iterator it = videoMap.find(textureId);
		// If found
		if (it != videoMap.end()) {
			ofVideoPlayer* video = it->second;
			video->update();
			std::map<GLuint, unsigned char*>::const_iterator itPix = videoPix.find(textureId);
			// If found
			if (itPix != videoPix.end()) {
				unsigned char* videoPixels = itPix->second;
				videoPixels = video->getPixels();
				glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, video->getWidth(), video->getHeight(), GL_RGB, GL_UNSIGNED_BYTE, videoPixels);
			}
		}
	}

	void TxManager::updateVideoTextures() {
		std::map<GLuint, ofVideoPlayer*>::const_iterator it;
		for (it = videoMap.begin(); it != videoMap.end(); it++) {
			bindTexture(it->first);
			ofVideoPlayer* video = it->second;
			video->update();
			std::map<GLuint, unsigned char*>::const_iterator itPix = videoPix.find(it->first);
			// If found
			if (itPix != videoPix.end()) {
				unsigned char* videoPixels = itPix->second;
				videoPixels = video->getPixels();
				glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, video->getWidth(), video->getHeight(), GL_RGB, GL_UNSIGNED_BYTE, videoPixels);
			}
		}
	}

	void TxManager::bindTexture(GLuint textureId) const {
		glBindTexture(GL_TEXTURE_2D, textureId);
	}

	void TxManager::unloadTexture(GLuint textureId) {
		// Check first if texture is associated to video
		std::map<GLuint, ofVideoPlayer*>::const_iterator it = videoMap.find(textureId);
		// If found
		if (it != videoMap.end()) {
			ofVideoPlayer* video = it->second;
			// Remove from map
			videoMap.erase(it);
			// Close video and delete
			video->closeMovie();
			delete video;
		}
		glDeleteTextures(1, &textureId);
	}

	void TxManager::enableTextures() const {
		glEnable(GL_TEXTURE_2D);
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
		// GL_REPLACE can be specified to just draw the surface using the texture colors only
		// GL_MODULATE means the computed surface color is multiplied by the texture color (to be used when lighting)
	}

	void TxManager::disableTextures() const {
		glDisable(GL_TEXTURE_2D);
	}

}
