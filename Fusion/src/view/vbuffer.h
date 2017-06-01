#ifndef VBUFFER_H
#define VBUFFER_H

#include <GL/glew.h>

#include <memory>

class vbuffer
{
public:
    vbuffer(const float* data, size_t count, GLenum type, size_t nprim = 0);
    ~vbuffer();

    void bind();
    void unbind();
    void draw(GLint first, GLsizei count);
    void draw();
    void update(const float* data, size_t count);

    void setup_attrib(GLint loc, GLint n, GLint offset, GLint size);

    GLuint id() const {
       return _id;
    }

    GLuint vao() const {
        return _vao;
    }

    GLuint vbo() const {
        return _id;
    }


    bool save(const char* p, size_t vsize);
    static std::shared_ptr<vbuffer> from_obj(const char* p);

private:
    float* _data;
    size_t _nb;
    size_t _count;
    GLuint _id;
    GLuint _vao;
    GLenum _type;
};

#endif // VBUFFER_H
