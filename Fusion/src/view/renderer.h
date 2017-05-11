#ifndef RENDERER_H
#define RENDERER_H

#include <memory>
#include <glm/glm.hpp>

#include "sensors/fetcher.h"
#include "sensors/crbuffer.h"

#include "view_window.h"
#include "vbuffer.h"
#include "shader.h"

class renderer
{
public:
    renderer(const view_window& window);

    void update(float dt);
    void render();

    void upload_points(fetcher& fetcher);

    glm::mat4 view;
    glm::vec3 eye;
private:
    glm::mat4 _proj;
    static const size_t MAX_POINTS_COUNT = 100000;

    crbuffer<glm::vec3, MAX_POINTS_COUNT * 2> _points_buf;
    std::shared_ptr<shader> _shader;
    std::shared_ptr<vbuffer> _points;
    std::shared_ptr<vbuffer> _plane;

    const view_window& _window;
};

#endif // RENDERER_H
