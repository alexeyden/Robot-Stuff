#include "shader.h"

#include <iostream>

#include <glm/gtc/type_ptr.hpp>

shader::shader(const char *frag, const char *vert)
{
    GLuint v = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(v, 1, &vert, nullptr);
    glCompileShader(v);

    GLint status;
    glGetShaderiv(v, GL_COMPILE_STATUS, &status);

    if(status == GL_FALSE) {
        char buffer[512];
        glGetShaderInfoLog(v, 512, NULL, buffer);
        std::cerr << "Vertex error: " << buffer << std::endl;
    }

    GLuint f = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(f, 1, &frag, nullptr);
    glCompileShader(f);

    glGetShaderiv(f, GL_COMPILE_STATUS, &status);

    if(status == GL_FALSE) {
        char buffer[512];
        glGetShaderInfoLog(f, 512, NULL, buffer);
        std::cerr << "Fragment error: " << buffer << std::endl;
    }

    GLuint p = glCreateProgram();
    glAttachShader(p, v);
    glAttachShader(p, f);

    glBindFragDataLocation(p, 0, "outColor");
    glLinkProgram(p);

    glDeleteShader(v);
    glDeleteShader(f);

    _program = p;
}

shader::~shader()
{
    glDeleteProgram(_program);
}

void shader::bind()
{
    glUseProgram(_program);
}

void shader::uniform1f(const char *name, float x)
{
    glUniform1f(glGetUniformLocation(_program, name), x);
}

void shader::uniform2f(const char *name, const glm::vec2 &v)
{
    glUniform2f(glGetUniformLocation(_program, name), v.x, v.y);
}

void shader::uniform3f(const char *name, const glm::vec3 &v)
{
    glUniform3f(glGetUniformLocation(_program, name), v.x, v.y, v.z);
}

void shader::uniform4f(const char *name, const glm::vec4 &v)
{
    glUniform4f(glGetUniformLocation(_program, name), v.x, v.y, v.z, v.w);
}

void shader::uniform1i(const char *name, GLint u)
{
    glUniform1i(glGetUniformLocation(_program, name), u);
}

void shader::uniform_mat4(const char *name, const glm::mat4 &m)
{
    glUniformMatrix4fv(glGetUniformLocation(_program, name), 1, GL_FALSE, glm::value_ptr(m));
}

GLint shader::get_attrib_location(const char *name) const
{
    return glGetAttribLocation(_program, name);
}
