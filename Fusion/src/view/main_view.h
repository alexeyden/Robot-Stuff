#ifndef MAIN_VIEW_H
#define MAIN_VIEW_H

#include <future>

#include "font.h"
#include "view.h"
#include "view_window.h"
#include "renderer.h"

#include "sensors/mesh.h"

#include <GLFW/glfw3.h>

class main_view : public view
{
public:
    main_view(view_window* parent);

    virtual void update(float dt) override;
    virtual void draw() override;

    virtual void on_mouse(float x, float y) override;
    virtual void on_key(int k, bool r) override;

    virtual void resize(int w, int h) override;
private:
    void update_mesh(const mesh_builder::data_t &points);

    float _time;
    bool _cursor;
    bool _light;

    glm::vec3 _pos;
    glm::vec3 _dir;
    glm::vec3 _up;
    float _angle_h;
    float _angle_v;

    glm::mat4 _font_proj;

    glm::vec2 _mouse;

    std::shared_ptr<font> _font;
    std::shared_ptr<shader> _2d_shader;
    mesh_builder _mesh_builder;

    renderer _renderer;
};

#endif // MAIN_VIEW_H
