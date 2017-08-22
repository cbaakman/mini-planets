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

#include <string>
#include <list>
#include <stdio.h>
#include <cstring>

#include "exception.h"
#include "shader.h"

void GetShaderTypeName(GLenum type, char *pOut)
{
    switch (type)
    {
    case GL_VERTEX_SHADER:
        strcpy(pOut, "vertex");
        break;
    case GL_FRAGMENT_SHADER:
        strcpy(pOut, "fragment");
        break;
    case GL_GEOMETRY_SHADER:
        strcpy(pOut, "geometry");
        break;
    case GL_TESS_CONTROL_SHADER:
        strcpy(pOut, "tessellation control");
        break;
    case GL_TESS_EVALUATION_SHADER:
        strcpy(pOut, "tessellation evaluation");
        break;
    default:
        strcpy(pOut, "");
    };
}

GLuint CreateShader(const std::string& source, GLenum type)
{
    char typeName[100];
    GetShaderTypeName(type, typeName);

    if (!typeName[0])
    {
        throw FormatableException("Unknown shader type: %.4X", type);
    }

    GLint result;
    int logLength;
    GLuint shader = glCreateShader(type);

    // Compile
    const char *src_ptr = source.c_str();
    glShaderSource(shader, 1, &src_ptr, NULL);
    glCompileShader(shader);

    glGetShaderiv(shader, GL_COMPILE_STATUS, &result);
    if (result != GL_TRUE)
    {
        // Error occurred, get compile log:
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logLength);

        char *errorString = new char[logLength + 1];
        glGetShaderInfoLog(shader, logLength, NULL, errorString);

        FormatableException e("Error compiling %s shader: %s", typeName, errorString);

        delete [] errorString;

        glDeleteShader(shader);

        throw e;
    }

    return shader;
}
GLuint CreateShaderProgram(const std::list <GLuint>& shaders);
GLuint CreateShaderProgram(const GLuint s1)
{
    std::list <GLuint> shaders;
    shaders.push_back(s1);
    return CreateShaderProgram(shaders);
}
GLuint CreateShaderProgram (const GLuint s1, const GLuint s2)
{
    std::list <GLuint> shaders;
    shaders.push_back(s1);
    shaders.push_back(s2);
    return CreateShaderProgram(shaders);
}
GLuint CreateShaderProgram(const GLuint s1, const GLuint s2, const GLuint s3)
{
    std::list <GLuint> shaders;
    shaders.push_back(s1);
    shaders.push_back(s2);
    shaders.push_back(s3);
    return CreateShaderProgram(shaders);
}
GLuint CreateShaderProgram(const GLuint s1, const GLuint s2, const GLuint s3,
                           const GLuint s4)
{
    std::list <GLuint> shaders;
    shaders.push_back(s1);
    shaders.push_back(s2);
    shaders.push_back(s3);
    shaders.push_back(s4);
    return CreateShaderProgram(shaders);
}
GLuint CreateShaderProgram(const GLuint s1, const GLuint s2, const GLuint s3,
                           const GLuint s4, const GLuint s5)
{
    std::list <GLuint> shaders;
    shaders.push_back(s1);
    shaders.push_back(s2);
    shaders.push_back(s3);
    shaders.push_back(s4);
    shaders.push_back(s5);
    return CreateShaderProgram(shaders);
}
GLuint CreateShaderProgram(const GLuint s1, const GLuint s2, const GLuint s3,
                           const GLuint s4, const GLuint s5, const GLuint s6)
{
    std::list <GLuint> shaders;
    shaders.push_back(s1);
    shaders.push_back(s2);
    shaders.push_back(s3);
    shaders.push_back(s4);
    shaders.push_back(s5);
    shaders.push_back(s6);
    return CreateShaderProgram(shaders);
}
GLuint CreateShaderProgram(const std::list <GLuint>& shaders)
{
    GLint result = GL_FALSE;
    int logLength;
    GLuint program = 0;

    // Combine the compiled shaders into a program:
    program = glCreateProgram();

    for (auto shader : shaders)
        glAttachShader(program, shader);

    glLinkProgram(program);

    glGetProgramiv(program, GL_LINK_STATUS, &result);
    if (result != GL_TRUE)
    {
        // Error occurred, get log:

        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &logLength);

        char *errorString = new char[logLength];
        glGetProgramInfoLog(program, logLength, NULL, errorString);

        FormatableException e("Error linking shader program: %s", errorString);

        delete [] errorString;

        glDeleteProgram(program);

        throw e;
    }

    return program;
}
GLuint CreateShaderProgram(GLenum type1, const std::string& source1)
{
    GLuint sh1 = CreateShader(source1, type1);
    if (!sh1)
    {
        glDeleteShader(sh1);
        return 0;
    }

    GLuint program = CreateShaderProgram(sh1);

    // schedule to delete when program is deleted
    glDeleteShader(sh1);

    return program;
}
GLuint CreateShaderProgram(GLenum type1, const std::string& source1,
                           GLenum type2, const std::string& source2)
{
    GLuint sh1 = CreateShader(source1, type1),
           sh2 = CreateShader(source2, type2);
    if (!(sh1 && sh2))
    {
        glDeleteShader(sh1);
        glDeleteShader(sh2);
        return 0;
    }

    GLuint program = CreateShaderProgram(sh1, sh2);

    // schedule to delete when program is deleted
    glDeleteShader(sh1);
    glDeleteShader(sh2);

    return program;
}
GLuint CreateShaderProgram(GLenum type1, const std::string& source1,
                           GLenum type2, const std::string& source2,
                           GLenum type3, const std::string& source3)
{
    GLuint sh1 = CreateShader(source1, type1),
           sh2 = CreateShader(source2, type2),
           sh3 = CreateShader(source3, type3);

    if (!(sh1 && sh2 && sh3))
    {
        glDeleteShader(sh1);
        glDeleteShader(sh2);
        glDeleteShader(sh3);
        return 0;
    }

    GLuint program = CreateShaderProgram(sh1, sh2, sh3);

    // schedule to delete when program is deleted
    glDeleteShader(sh1);
    glDeleteShader(sh2);
    glDeleteShader(sh3);

    return program;
}
GLuint CreateShaderProgram(GLenum type1, const std::string& source1,
                           GLenum type2, const std::string& source2,
                           GLenum type3, const std::string& source3,
                           GLenum type4, const std::string& source4)
{
    GLuint sh1 = CreateShader(source1, type1),
           sh2 = CreateShader(source2, type2),
           sh3 = CreateShader(source3, type3),
           sh4 = CreateShader(source4, type4);

    if (!(sh1 && sh2 && sh3 && sh4))
    {
        glDeleteShader(sh1);
        glDeleteShader(sh2);
        glDeleteShader(sh3);
        glDeleteShader(sh4);
        return 0;
    }

    GLuint program = CreateShaderProgram(sh1, sh2, sh3, sh4);

    // schedule to delete when program is deleted
    glDeleteShader(sh1);
    glDeleteShader(sh2);
    glDeleteShader(sh3);
    glDeleteShader(sh4);

    return program;
}
GLuint CreateShaderProgram(GLenum type1, const std::string& source1,
                           GLenum type2, const std::string& source2,
                           GLenum type3, const std::string& source3,
                           GLenum type4, const std::string& source4,
                           GLenum type5, const std::string& source5)
{
    GLuint sh1 = CreateShader(source1, type1),
           sh2 = CreateShader(source2, type2),
           sh3 = CreateShader(source3, type3),
           sh4 = CreateShader(source4, type4),
           sh5 = CreateShader(source5, type5);

    if (!(sh1 && sh2 && sh3 && sh4 && sh5))
    {
        glDeleteShader(sh1);
        glDeleteShader(sh2);
        glDeleteShader(sh3);
        glDeleteShader(sh4);
        glDeleteShader(sh5);
        return 0;
    }

    GLuint program = CreateShaderProgram(sh1, sh2, sh3, sh4, sh5);

    // schedule to delete when program is deleted
    glDeleteShader(sh1);
    glDeleteShader(sh2);
    glDeleteShader(sh3);
    glDeleteShader(sh4);
    glDeleteShader(sh5);

    return program;
}
