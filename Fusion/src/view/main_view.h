#ifndef MAIN_VIEW_H
#define MAIN_VIEW_H

#include "font.h"
#include "view.h"
#include "view_window.h"
#include "renderer.h"

#include <GLFW/glfw3.h>

class main_view : public view
{
public:
    main_view(view_window* parent);

    virtual void update(float dt) override;
    virtual void draw() override;

    virtual void on_mouse(float x, float y) override;
    virtual void on_key(int k, bool r) override;

private:
    float _time;
    bool _cursor;

    glm::vec3 _pos;
    glm::vec3 _dir;
    glm::vec3 _up;
    float _angle_h;
    float _angle_v;

    glm::mat4 _font_proj;

    glm::vec2 _mouse;

    std::shared_ptr<font> _font;
    std::shared_ptr<shader> _2d_shader;

    renderer _renderer;
};

#endif // MAIN_VIEW_H
