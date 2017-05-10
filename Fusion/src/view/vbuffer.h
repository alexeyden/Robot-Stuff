#ifndef VBUFFER_H
#define VBUFFER_H

#include <GL/glew.h>

class vbuffer
{
public:
    vbuffer(const float* data, size_t count, GLenum type);
    ~vbuffer();

    void bind();
    void draw(GLint first, GLsizei count);

    void setup_attrib(GLint loc, GLint n, GLint offset, GLint size);

    GLuint id() const {
       return _id;
    }

    GLuint vao() const {
        return _vao;
    }

private:
    GLuint _id;
    GLuint _vao;
    GLenum _type;
};

#endif // VBUFFER_H
