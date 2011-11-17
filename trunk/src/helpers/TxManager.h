#ifndef TX_MANAGER_H__
#define TX_MANAGER_H__

#include "ITxManager.h"

#include "ofxFenster.h"

namespace mapinect {

	class TxManager : public ITxManager {
	public:
		TxManager(ofxFenster* f);

		GLuint	loadTexture(string imgFile);
		void	unloadTexture(GLuint textureId);

		void	bindTexture(GLuint textureId) const;
		
		void	enableTextures() const;
		void	disableTextures() const;

	private:
		ofxFenster* fenster;

	};
}

#endif	// TX_MANAGER_H__
