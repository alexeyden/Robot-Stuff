#ifndef SHADER_H
#define SHADER_H

#include <GL/glew.h>
#include <glm/glm.hpp>

class shader
{
public:
    shader(const char* frag, const char* vert);
    ~shader();

    void bind();

    void uniform1f(const char* name, float x);
    void uniform2f(const char* name, const glm::vec2& v);
    void uniform3f(const char* name, const glm::vec3& v);
    void uniform4f(const char* name, const glm::vec4& v);
    void uniform1i(const char* name, GLint u);
    void uniform_mat4(const char* name, const glm::mat4& m);

    GLint get_attrib_location(const char* name) const;

private:
    GLuint _program;
};

#endif // SHADER_H
