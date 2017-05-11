#include "vbuffer.h"

#include <cstring>

vbuffer::vbuffer(const float *data, size_t count, GLenum type) :
    _type(type)
{
    glGenVertexArrays(1, &_vao);
    glBindVertexArray(_vao);

    glGenBuffers(1, &_id);
    glBindBuffer(GL_ARRAY_BUFFER, _id);

    update(data, count);
}

vbuffer::~vbuffer()
{
    glDeleteBuffers(1, &_id);
    glDeleteVertexArrays(1, &_vao);
}

void vbuffer::bind()
{
    glBindVertexArray(_vao);
}

void vbuffer::draw(GLint first, GLsizei count)
{
    glDrawArrays(_type, first, count);
}

void vbuffer::update(const float *data, size_t count)
{
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * count, data, GL_STATIC_DRAW);
}

void vbuffer::setup_attrib(GLint loc, GLint n, GLint offset, GLint size)
{
    glEnableVertexAttribArray(loc);
    glVertexAttribPointer(loc, n, GL_FLOAT, GL_FALSE,
                          sizeof(float) * size,
                          (void*) (sizeof(float) * offset));
}
