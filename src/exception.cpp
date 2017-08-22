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

#include <sstream>
#include <cstdio>
#include <stdarg.h>

#include "exception.h"


FileOpenException::FileOpenException(const std::string &path,
                                     const std::string &error)
{
    std::ostringstream ss;

    ss << "error opening \"" << path << "\": " << error;

    mMessage = ss.str();
}
const char* FileOpenException::what() const throw ()
{
    return mMessage.c_str();
}
ParsingException::ParsingException(const std::string &path,
                                   const std::string &error)
{
    std::ostringstream ss;

    ss << "error parsing \"" << path << "\": " << error;

    mMessage = ss.str();
}
const char* ParsingException::what() const throw ()
{
    return mMessage.c_str();
}
std::string GLErrorString(const GLenum err)
{
    std::ostringstream ss;
    switch (err)
    {
    case GL_NO_ERROR:
        ss << "GL_NO_ERROR";
        break;
    case GL_INVALID_ENUM:
        ss << "GL_INVALID_ENUM";
        break;
    case GL_INVALID_VALUE:
        ss << "GL_INVALID_VALUE";
        break;
    case GL_INVALID_OPERATION:
        ss << "GL_INVALID_OPERATION";
        break;
    case GL_STACK_OVERFLOW:
        ss << "GL_STACK_OVERFLOW";
        break;
    case GL_STACK_UNDERFLOW:
        ss << "GL_STACK_UNDERFLOW";
        break;
    case GL_OUT_OF_MEMORY:
        ss << "GL_OUT_OF_MEMORY";
        break;
    case GL_INVALID_FRAMEBUFFER_OPERATION:
        ss << "GL_INVALID_FRAMEBUFFER_OPERATION";
        break;
    case GL_TABLE_TOO_LARGE:
        ss << "GL_TABLE_TOO_LARGE";
        break;
    default:
        ss << "OpenGL error " << std::hex << err;
        break;
    }

    return ss.str();
}
GLException::GLException(const std::string &doing, const GLenum err)
{
    mMessage = GLErrorString(err) + " while " + doing;
}
const char* GLException::what() const throw ()
{
    return mMessage.c_str();
}
std::string GLFrameBufferErrorString(const GLenum err)
{
    std::ostringstream ss;
    switch (err)
    {
    case GL_FRAMEBUFFER_UNDEFINED:
        ss << "GL_FRAMEBUFFER_UNDEFINED";
        break;
    case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
        ss << "GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT";
        break;
    case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
        ss << "GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT";
        break;
    case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
        ss << "GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER";
        break;
    case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
        ss << "GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER";
        break;
    case GL_FRAMEBUFFER_UNSUPPORTED:
        ss << "GL_FRAMEBUFFER_UNSUPPORTED";
        break;
    case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:
        ss << "GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE";
        break;
    case GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS:
        ss << "GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS";
        break;
    default:
        ss << "OpenGL Framebuffer error " << std::hex << err;
        break;
    }

    return ss.str();
}
GLFramebufferException::GLFramebufferException(const std::string &doing, const GLenum err)
{
    mMessage = GLFrameBufferErrorString(err) + " while " + doing;
}
const char* GLFramebufferException::what() const throw ()
{
    return mMessage.c_str();
}
FormatableException::FormatableException(const char *format, ...)
{
    va_list args;

    va_start(args, format);

    vsnprintf (message, sizeof(message), format, args);

    va_end(args);
}
const char* FormatableException::what() const throw ()
{
    return message;
}
