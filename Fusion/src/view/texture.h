#ifndef TEXTURE_H
#define TEXTURE_H

#include <cstdint>
#include <GL/glew.h>

#include "image.h"

template<typename Item = uint8_t>
class texture
{
public:
    static texture<Item>* from_image(const image<Item>* img)
    {
        GLuint id;
        glGenTextures(1, &id);

        glBindTexture(GL_TEXTURE_2D, id);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

        texture<Item>* tex = new texture<Item>(id, img->width(), img->height());
        tex->update(img);

        return tex;
    }
    ~texture()
    {
        glDeleteTextures(1, &_id);
    }

    size_t width() const {
        return _width;
    }

    size_t height() const {
        return _height;
    }

    GLuint id() const {
        return _id;
    }

    void bind()
    {
        glBindTexture(GL_TEXTURE_2D, _id);
    }

    void update(const image<Item>* img)
    {
        GLenum intern_format = GL_RGB;
        GLenum format = GL_RGB;
        if(img->channels() == 1) {
            format = GL_R8;
            intern_format = GL_RED;
        }
        else if(img->channels() == 3) {
            format = GL_RGB;
            intern_format = GL_RGB;
        }
        else if(img->channels() == 4) {
            format = GL_RGBA;
            intern_format = GL_RGBA;
        }

        GLenum item_type = GL_UNSIGNED_BYTE;
        if(std::is_same<Item, uint8_t>::value) {
            item_type = GL_UNSIGNED_BYTE;
        }
        else if(std::is_same<Item, float>::value) {
            item_type = GL_FLOAT;
        }

        glTexImage2D(GL_TEXTURE_2D, 0, format, img->width(), img->height(), 0, intern_format, item_type, img->data());
    }

private:
    texture(GLuint id, size_t width, size_t height) :
        _id(id), _width(width), _height(height)
    {
    }

    GLuint _id;
    size_t _width;
    size_t _height;
};

#endif // TEXTURE_H
