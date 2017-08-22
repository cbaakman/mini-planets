/* Copyright (C) 2017 Coos Baakman
  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.
  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:
  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

#ifndef TEXTURE_H
#define TEXTURE_H

#include <png.h>
#include <string>

#include <GL/glew.h>
#include <GL/gl.h>

#include <SDL2/SDL.h>


#define PNG_LOAD_ERROR_ID "png-load-error"


struct Texture {

    GLuint tex;
    GLsizei w, h;
};

struct CubeMap {

    GLuint tex;
};

/**
 * Loads a png image from SDL_RWops as an OpenGL texture.
 */
void LoadPNG(SDL_RWops *, Texture *);

void LoadPNGs(SDL_RWops *pXneg, SDL_RWops *pXpos,
              SDL_RWops *pYneg, SDL_RWops *pYpos,
              SDL_RWops *pZneg, SDL_RWops *pZpos, CubeMap *);

#endif // TEXTURE_H
