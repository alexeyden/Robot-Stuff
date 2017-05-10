#ifndef FONT_H
#define FONT_H

#include <vector>
#include <memory>

#include <GL/glew.h>

#include "shader.h"
#include "texture.h"
#include "vbuffer.h"

class font
{
public:
    font(const char* path, std::shared_ptr<shader> sh, float size, size_t chw, size_t chh);
    ~font();

    void draw(const unsigned char* string, float x, float y);

    glm::vec3 color;
    float size;
private:
    uint32_t _chw;
    uint32_t _chh;

    std::shared_ptr<texture<>> _texture;
    std::shared_ptr<shader> _shader;
    std::shared_ptr<vbuffer> _vbo;
};

#endif // FONT_H
