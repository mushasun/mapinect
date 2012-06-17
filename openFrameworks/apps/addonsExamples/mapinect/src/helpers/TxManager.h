#ifndef TX_MANAGER_H__
#define TX_MANAGER_H__

#include "ITxManager.h"

#include <map>
#include "ofVideoPlayer.h"

namespace mapinect {

	class TxManager : public ITxManager {
	public:
		TxManager();

		GLuint	loadImageTexture(string imgFile);
		
		GLuint	loadVideoTexture(string videoFile);
		void	updateVideoTextures();

		void	bindTexture(GLuint textureId) const;
		void	unloadTexture(GLuint textureId);
		
		void	enableTextures() const;
		void	disableTextures() const;

	private:
		std::map<GLuint,ofVideoPlayer*> videoMap;
		std::map<GLuint,unsigned char*> videoPix;

		void	updateVideoTexture(GLuint textureId);

	};
}

#endif	// TX_MANAGER_H__
