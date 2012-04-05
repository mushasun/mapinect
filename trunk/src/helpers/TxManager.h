#ifndef TX_MANAGER_H__
#define TX_MANAGER_H__

#include "ITxManager.h"
#include "ofxFenster.h"

namespace mapinect {

	class TxManager : public ITxManager {
	public:
		TxManager(ofxFenster* f);

		GLuint	loadImageTexture(string imgFile);
		
		GLuint	loadVideoTexture(string videoFile);
		void	updateVideoTexture(GLuint textureId);

		void	bindTexture(GLuint textureId) const;
		void	unloadTexture(GLuint textureId);
		
		void	enableTextures() const;
		void	disableTextures() const;

	private:
		ofxFenster* fenster;
		std::map<GLuint,ofVideoPlayer*> videoMap;
		std::map<GLuint,unsigned char*> videoPix;
		
	};
}

#endif	// TX_MANAGER_H__
