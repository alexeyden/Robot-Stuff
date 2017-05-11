#ifndef VIEW_DEBUG_H
#define VIEW_DEBUG_H

#include <memory>
#include <glm/glm.hpp>

#include "view.h"
#include "font.h"
#include "shader.h"
#include "vbuffer.h"

class view_window;

class view_debug : public view
{
public:
    view_debug(view_window* parent);
    virtual ~view_debug();

    virtual void update(float dt);
    virtual void draw();

    virtual void resize(int w, int h) override;

private:
    void setup_lidar();
    void setup_cameras();

    glm::mat4 _proj;
    std::shared_ptr<font> _font;
    std::shared_ptr<shader> _2d_shader;

    std::shared_ptr<vbuffer> _quad_buf;
    std::shared_ptr<texture<float>> _lidar_tex;
    std::shared_ptr<texture<>> _cam_left_tex;
    std::shared_ptr<texture<>> _cam_right_tex;

    float _update_time;
};

#endif // VIEW_DEBUG_H
