#include "TxManager.h"

namespace mapinect {

	TxManager::TxManager(ofxFenster* f) {
		fenster = f;
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

	void TxManager::bindTexture(GLuint textureId) const {
		glBindTexture(GL_TEXTURE_2D, textureId);
	}

	GLuint TxManager::loadTexture(string imgFile)
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

	void TxManager::unloadTexture(GLuint textureId) {
		glDeleteTextures(1, &textureId);
	}

}
