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

#ifndef SHADER_H
#define SHADER_H

#include <GL/glew.h>
#include <GL/gl.h>

#include <string>

/**
 * Compile the source of a shader.
 * :param type: GL_VERTEX_SHADER, GL_TESS_CONTROL_SHADER, GL_TESS_EVALUATION_SHADER, GL_GEOMETRY_SHADER, or GL_FRAGMENT_SHADER.
 * :returns: OpenGL handle to shader, or 0 on error
 */
GLuint CreateShader(const std::string& source, GLenum type);

/**
 * Creates a shader program from given shaders.
 * :param shaderN: shader objects created with 'CreateShader', max 6
 * :returns: OpenGL handle to the shader program, or 0 on error
 */
GLuint CreateShaderProgram(const GLuint shader1);
GLuint CreateShaderProgram(const GLuint shader1, const GLuint shader2);
GLuint CreateShaderProgram(const GLuint shader1, const GLuint shader2, const GLuint shader3);
GLuint CreateShaderProgram(const GLuint shader1, const GLuint shader2, const GLuint shader3,
                           const GLuint shader4);
GLuint CreateShaderProgram(const GLuint shader1, const GLuint shader2, const GLuint shader3,
                           const GLuint shader4, const GLuint shader5);
GLuint CreateShaderProgram(const GLuint shader1, const GLuint shader2, const GLuint shader3,
                           const GLuint shader4, const GLuint shader5, const GLuint shader6);

// Shortcuts
GLuint CreateShaderProgram(GLenum type1, const std::string& source1);
GLuint CreateShaderProgram(GLenum type1, const std::string& source1,
                           GLenum type2, const std::string& source2);
GLuint CreateShaderProgram(GLenum type1, const std::string& source1,
                           GLenum type2, const std::string& source2,
                           GLenum type3, const std::string& source3);
GLuint CreateShaderProgram(GLenum type1, const std::string& source1,
                           GLenum type2, const std::string& source2,
                           GLenum type3, const std::string& source3,
                           GLenum type4, const std::string& source4);
GLuint CreateShaderProgram(GLenum type1, const std::string& source1,
                           GLenum type2, const std::string& source2,
                           GLenum type3, const std::string& source3,
                           GLenum type4, const std::string& source4,
                           GLenum type5, const std::string& source5);

#endif // SHADER_H
