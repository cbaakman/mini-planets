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

#ifndef EXCEPTION_H
#define EXCEPTION_H

#include <exception>
#include <string>

#include <GL/glew.h>
#include <GL/gl.h>


class FileOpenException : public std::exception
{
private:
    std::string mMessage;
public:
    FileOpenException(const std::string &path,
                      const std::string &error);

    const char* what() const throw ();
};

class ParsingException : public std::exception
{
private:
    std::string mMessage;
public:
    ParsingException(const std::string &path,
                     const std::string &error);

    const char* what() const throw ();
};

std::string GLErrorString(const GLenum err);

class GLException : public std::exception
{
private:
    std::string mMessage;
public:
    GLException(const std::string &doingWhat, const GLenum err);

    const char* what() const throw ();
};

std::string GLFrameBufferErrorString(const GLenum err);

class GLFramebufferException : public std::exception
{
private:
    std::string mMessage;
public:
    GLFramebufferException(const std::string &doingWhat, const GLenum err);

    const char* what() const throw ();
};

class FormatableException : public std::exception
{
private:
    char message[16384];
public:
    FormatableException(const char *format, ...);

    const char* what() const throw ();
};

#endif // EXCEPTION_H
