#include "font.h"

#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "image.h"

font::font(const char *path, std::shared_ptr<shader> sh, float size, size_t chw, size_t chh) :
    color(0.0f, 0.0f, 0.0f), size(size), _chw(chw), _chh(chh), _shader(sh)
{
    image<>* img = image<>::load(path);

    size_t data_size = (2 + 2) * 4 * img->width() / chw * img->height() / chh;
    float* data = new float[data_size];

    for(size_t y = 0; y < img->height()/chh; y += 1) {
      for(size_t x = 0; x < img->width()/chw; x += 1) {
            float* dp = data + (2 + 2) * 4 * (y * img->width()/chw + x);
            float u0 = x * chw / (float) img->width();
            float v0 = y * chh / (float) img->height();
            float u1 = (x * chw + chw) / (float) img->width();
            float v1 = (y * chh + chh) / (float) img->height();

            dp[0]  = 0.0f; dp[1]  = chh;  dp[2]  = u0; dp[3]  = v1;
            dp[4]  = 0.0f; dp[5]  = 0.0f; dp[6]  = u0; dp[7]  = v0;
            dp[8]  = chw;  dp[9]  = chh;  dp[10] = u1; dp[11] = v1;
            dp[12] = chw;  dp[13] = 0.0f; dp[14] = u1; dp[15] = v0;
        }
    }

    _texture = std::shared_ptr<texture<>>(texture<>::from_image(img));
    _vbo = std::shared_ptr<vbuffer>(new vbuffer(data, data_size, GL_TRIANGLE_STRIP));

    GLint pos_loc = _shader->get_attrib_location("position");
    GLint uv_loc = _shader->get_attrib_location("uvp");

    _vbo->setup_attrib(pos_loc, 2, 0, 4);
    _vbo->setup_attrib(uv_loc, 2, 2, 4);

    delete [] data;
    delete img;
}

font::~font()
{
}

void font::draw(const unsigned char *string, float x, float y)
{
    _shader->uniform3f("color", color);
    _shader->uniform1i("tex", 0);

    _vbo->bind();
    _texture->bind();

    float xa = 0.0f;
    float ya = 0.0f;

    while(*string != 0) {
        if(*string == '\n') {
            ya += _chh;
            xa = 0.0f;
            string++;
            continue;
        }

        _shader->uniform_mat4("model", glm::translate(glm::mat4(1.0f), glm::vec3(x + xa, y + ya, 0.0f)));
        _vbo->draw(*string * 4, 4);

        xa += _chw;
        string++;
    }
}
