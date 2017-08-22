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

#include <functional>

#include "texture.h"
#include "exception.h"
#include "log.h"


void PNGReadCallback(png_structp png_ptr, png_bytep data, png_size_t length)
{
    /*
        Image data comes from an SDL_RWops input stream.
        This callback function gets the requested bytes from it.
     */

    png_voidp pio = png_get_io_ptr(png_ptr);
    SDL_RWops *io = (SDL_RWops *)pio;

    size_t n;
    if ((n = io->read(io, data, 1, length)) < length)
    {
        throw FormatableException("%d bytes requested, only %d read", length, n);
    }
}
void PNGErrorCallback(png_structp png_ptr, png_const_charp msg)
{
    /*
       PNG errors must be thrown as an exception,
       not just stderr.
     */

    throw FormatableException("Error from png: %s", (const char *)msg);
}
void PNGWarningCallback(png_structp png_ptr, png_const_charp msg)
{
    // Not decided what to do with warnings yet.
}

typedef std::function<void (const png_bytep png_data, const png_uint_32 png_width, const png_uint_32 png_height, const int color_type)> GLTextureFunc;

void LoadPNG(SDL_RWops *io, GLTextureFunc texFunc)
{
    /*
        We use libpng to read in a png file.
        For further information:
        http://www.libpng.org/pub/png/libpng-1.2.5-manual.html
     */

    png_info* info_ptr = 0, *end_info = 0;
    png_struct* png_ptr = 0;
    png_bytep *row_pointers = 0;
    png_bytep png_data = 0;
    png_uint_32 rowbytes;
    int number_of_passes;
    png_byte num_channels;

    try
    {
        // make a read structure:
        png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, (png_voidp)NULL, NULL, NULL);
        if (!png_ptr)
        {
            throw FormatableException(
                       "png: error creating read struct");
        }

        // info and end_info structures are also neccesary:
        info_ptr = png_create_info_struct(png_ptr);
        if (!info_ptr)
        {
            throw FormatableException(
                       "png: error creating info struct");
        }
        end_info = png_create_info_struct(png_ptr);
        if (!end_info)
        {
            throw FormatableException(
                       "png: error creating end info");
        }

        // Catch libPNG's errors and warnings:
        png_set_error_fn(png_ptr, png_get_error_ptr(png_ptr), PNGErrorCallback, PNGWarningCallback);

        // Tell libpng that it must take its data from a callback function:
        png_set_read_fn(png_ptr, (png_voidp *)io, PNGReadCallback);

        // read info from the header:
        png_read_info(png_ptr, info_ptr);

        // Get info about the data in the image:
        png_uint_32 png_width, png_height;
        int bit_depth,
            color_type,
            interlace_type;

        png_get_IHDR(png_ptr, info_ptr, &png_width, &png_height, &bit_depth, &color_type, &interlace_type, NULL, NULL);

        // how many channels does the image have?
        num_channels = png_get_channels(png_ptr, info_ptr);

        // We want RGB colors, no palette:
        if (color_type == PNG_COLOR_TYPE_PALETTE)
        {
            png_set_palette_to_rgb(png_ptr);
            color_type = PNG_COLOR_TYPE_RGB;
        }

        // make sure the pixel depth is 8:
        if (bit_depth < 8)
        {
            png_set_packing(png_ptr);
            bit_depth = 8;
        }
        else if (bit_depth == 16)
        {
            png_set_strip_16(png_ptr);
            bit_depth = 8;
        }

        // Make sure the transparency is set to an alpha channel, not tRNS:
        if (png_get_valid(png_ptr, info_ptr, PNG_INFO_tRNS))
            png_set_tRNS_to_alpha(png_ptr);

        // Handle expansion of interlaced image:
        number_of_passes = png_set_interlace_handling(png_ptr);

        // Update  png_info structure after setting transformations:
        png_read_update_info(png_ptr, info_ptr);

        // get bytes per row:
        rowbytes = png_get_rowbytes(png_ptr, info_ptr);

        // allocate memory to store pixel data in:
        png_data = (png_bytep)png_malloc(png_ptr, png_height * rowbytes);
        if (!png_data)
        {
            throw FormatableException("png_malloc failed");
        }

        // make row pointers for reading:
        row_pointers = new png_bytep[png_height];

        // place the row pointers correctly:
        for(png_uint_32 i = 0; i < png_height; i++)
            row_pointers[png_height - 1 - i] = png_data + i * rowbytes;

        png_read_image(png_ptr, row_pointers);

        // Don't need row pointers anymore, png_data has been filled up.
        delete [] row_pointers;

        // read the end of the file:
        png_read_end(png_ptr, end_info);

        texFunc(png_data, png_width, png_height, color_type);

        png_free(png_ptr, png_data);
        png_data = 0;

        // clean up (also frees whatever was allocated with 'png_malloc'):
        png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
        png_ptr = 0;
        info_ptr = 0;
        end_info = 0;
    }
    catch (std::exception &e)
    {
        if (png_data)
            png_free(png_ptr, png_data);

        delete [] row_pointers; // free allocated row-pointers, if any
        png_destroy_read_struct(&png_ptr, &info_ptr, &end_info); // clean up the read structure, info and end_info

        throw FormatableException("error parsing png data, %s", e.what());
    }
}
GLenum ColorPNG2GL(const int color_type)
{
    GLenum format;
    switch (color_type)
    {
    case PNG_COLOR_TYPE_GRAY:
        format = GL_LUMINANCE8;
        break;
    case PNG_COLOR_TYPE_GRAY_ALPHA:
        format = GL_LUMINANCE8_ALPHA8;
        break;
    case PNG_COLOR_TYPE_RGB:
        format = GL_RGB;
        break;
    case PNG_COLOR_TYPE_RGB_ALPHA:
        format = GL_RGBA;
        break;
    }
    return format;
}
void LoadPNG(SDL_RWops *io, Texture *pTex)
{
    LoadPNG(io,
        [pTex](const png_bytep png_data, const png_uint_32 png_width, const png_uint_32 png_height, const int color_type)
        {
            GLenum format = ColorPNG2GL(color_type);

            // Create an OpenGL texture:
            GLuint tex;
            glGenTextures(1, &tex);
            if (!tex)
            {
                throw GLException("generating GL texture for png", glGetError());
            }

            glBindTexture(GL_TEXTURE_2D, tex);

            glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

            glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

            // Make OpenGL fill the texture:
            glTexImage2D(GL_TEXTURE_2D, 0, format, (GLsizei)png_width, (GLsizei)png_height, 0, format, GL_UNSIGNED_BYTE, png_data);
            glGenerateMipmap(GL_TEXTURE_2D);

            pTex->tex = tex;
            pTex->w = (GLsizei)png_width;
            pTex->h = (GLsizei)png_height;

            glBindTexture(GL_TEXTURE_2D, NULL);
        }
    );
}
void LoadPNGs(SDL_RWops *pXneg, SDL_RWops *pXpos,
              SDL_RWops *pYneg, SDL_RWops *pYpos,
              SDL_RWops *pZneg, SDL_RWops *pZpos, CubeMap *pMap)
{
    glGenTextures(1, &pMap->tex);
    if (!pMap->tex)
    {
        throw GLException("generating GL texture for png", glGetError());
    }
    glBindTexture(GL_TEXTURE_CUBE_MAP, pMap->tex);

    LoadPNG(pXneg,
        [pMap](const png_bytep png_data, const png_uint_32 png_width, const png_uint_32 png_height, const int color_type)
        {
            GLenum format = ColorPNG2GL(color_type);

            glTexImage2D(
                GL_TEXTURE_CUBE_MAP_NEGATIVE_X,
                0, format, (GLsizei)png_width, (GLsizei)png_height, 0, format, GL_UNSIGNED_BYTE, png_data
            );
        }
    );
    LoadPNG(pXpos,
        [pMap](const png_bytep png_data, const png_uint_32 png_width, const png_uint_32 png_height, const int color_type)
        {
            GLenum format = ColorPNG2GL(color_type);

            glTexImage2D(
                GL_TEXTURE_CUBE_MAP_POSITIVE_X,
                0, format, (GLsizei)png_width, (GLsizei)png_height, 0, format, GL_UNSIGNED_BYTE, png_data
            );
        }
    );
    LoadPNG(pYneg,
        [pMap](const png_bytep png_data, const png_uint_32 png_width, const png_uint_32 png_height, const int color_type)
        {
            GLenum format = ColorPNG2GL(color_type);

            glTexImage2D(
                GL_TEXTURE_CUBE_MAP_NEGATIVE_Y,
                0, format, (GLsizei)png_width, (GLsizei)png_height, 0, format, GL_UNSIGNED_BYTE, png_data
            );
        }
    );
    LoadPNG(pYpos,
        [pMap](const png_bytep png_data, const png_uint_32 png_width, const png_uint_32 png_height, const int color_type)
        {
            GLenum format = ColorPNG2GL(color_type);

            glTexImage2D(
                GL_TEXTURE_CUBE_MAP_POSITIVE_Y,
                0, format, (GLsizei)png_width, (GLsizei)png_height, 0, format, GL_UNSIGNED_BYTE, png_data
            );
        }
    );
    LoadPNG(pZneg,
        [pMap](const png_bytep png_data, const png_uint_32 png_width, const png_uint_32 png_height, const int color_type)
        {
            GLenum format = ColorPNG2GL(color_type);

            glTexImage2D(
                GL_TEXTURE_CUBE_MAP_NEGATIVE_Z,
                0, format, (GLsizei)png_width, (GLsizei)png_height, 0, format, GL_UNSIGNED_BYTE, png_data
            );
        }
    );
    LoadPNG(pZpos,
        [pMap](const png_bytep png_data, const png_uint_32 png_width, const png_uint_32 png_height, const int color_type)
        {
            GLenum format = ColorPNG2GL(color_type);

            glTexImage2D(
                GL_TEXTURE_CUBE_MAP_POSITIVE_Z,
                0, format, (GLsizei)png_width, (GLsizei)png_height, 0, format, GL_UNSIGNED_BYTE, png_data
            );
        }
    );
    glGenerateMipmap(GL_TEXTURE_CUBE_MAP);

    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE); 

    glBindTexture(GL_TEXTURE_CUBE_MAP, NULL);
}
