#ifndef I_TX_MANAGER_H__
#define I_TX_MANAGER_H__

/// <summary>
/// Interface for texture manager. Loading, unloading, binding and enabling
/// of user's application textures should be performed through this interface.
/// </summary>

#include "ofGraphics.h"

namespace mapinect {
	class ITxManager {
	public:

		virtual GLuint	loadTexture(string imgFile) = 0;
		virtual GLuint	loadVideoTexture(string videoFile) = 0;
		virtual void	updateVideoTexture() = 0;

		virtual void	unloadTexture(GLuint textureId) = 0;

		virtual void	bindTexture(GLuint textureId) const = 0;
		
		virtual void	enableTextures() const = 0;
		virtual void	disableTextures() const = 0;
	};
}

#endif	// I_TX_MANAGER_H__
