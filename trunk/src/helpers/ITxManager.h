#ifndef I_TX_MANAGER_H__
#define I_TX_MANAGER_H__

#include "ofGraphics.h"

namespace mapinect {
	class ITxManager {
	public:

		virtual GLuint	loadTexture(string imgFile) = 0;
		virtual void	unloadTexture(GLuint textureId) = 0;

		virtual void	bindTexture(GLuint textureId) const = 0;
		
		virtual void	enableTextures() const = 0;
		virtual void	disableTextures() const = 0;
	};
}

#endif	// I_TX_MANAGER_H__
