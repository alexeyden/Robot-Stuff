#include "renderer.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/random.hpp>

static const char* vert_shadow_src = "#version 150\n"
                                     "in vec3 position;"
                                     "in vec3 norm;"
                                     "uniform mat4 proj;"
                                     "void main() {"
                                     "gl_Position = proj * vec4(position, 1.0);"
                                     "}";

static const char* frag_dummy_src = "#version 150\n"
                                     "out vec4 outColor;"
                                     "void main() {"
                                     "outColor = vec4(1.0);"
                                     "}";

extern const char* SHADER_3D_FS;
extern const char* SHADER_3D_VS;
extern const char* SHADER_CLOUD_FS;
extern const char* SHADER_CLOUD_VS;

renderer::renderer(const view_window &window) :
    _window(window)
{
    _shader = std::shared_ptr<shader>(new shader(SHADER_3D_FS, SHADER_3D_VS));
    _point_shader = std::shared_ptr<shader>(new shader(SHADER_CLOUD_FS, SHADER_CLOUD_VS));
    _shadow_shader = std::shared_ptr<shader>(new shader(frag_dummy_src, vert_shadow_src));

    view = glm::mat4();
    _proj = glm::perspective(glm::radians(45.0f), _window.width() / (float) _window.height(), 0.1f, 100.0f);

    float plane[] = {
        -50.0f,  50.0f, 0.0f, 0.0f, 0.0f, 1.0f,
        -50.0f, -50.0f, 0.0f, 0.0f, 0.0f, 1.0f,
         50.0f,  50.0f, 0.0f, 0.0f, 0.0f, 1.0f,
         50.0f, -50.0f, 0.0f, 0.0f, 0.0f, 1.0f
    };

    _shader->bind();
    _plane = std::shared_ptr<vbuffer>(new vbuffer(plane, 4 * 6, GL_TRIANGLE_STRIP));
    _plane->setup_attrib(_shader->get_attrib_location("position"), 3, 0, 6);
    _plane->setup_attrib(_shader->get_attrib_location("norm"), 3, 3, 6);
    _plane->unbind();

    _mesh = std::shared_ptr<vbuffer>(new vbuffer(nullptr, 0, GL_TRIANGLES));
    _mesh->setup_attrib(_shader->get_attrib_location("position"), 3, 0, 6);
    _mesh->setup_attrib(_shader->get_attrib_location("norm"), 3, 3, 6);
    _mesh->unbind();

    init_shadows();

    float tmp[] = { 0.0f, 0.0f, 1.0f, 0.0f };
    _point_shader->bind();
    _points = std::shared_ptr<vbuffer>(new vbuffer(tmp, 4, GL_POINTS));
    _points->setup_attrib(_point_shader->get_attrib_location("position"), 3, 0, 4);
    _points->setup_attrib(_point_shader->get_attrib_location("prob"), 1, 3, 4);
    _points->unbind();

    _points_n = 1;
    _verts_n = 0;

    thresh = 0.0f;
}

void renderer::update(float dt)
{
    (void) dt;
}

void renderer::render()
{
    glm::vec3 lp0 = glm::vec3(10.0f, 10.0f, 10.0f);
    glm::vec3 lp1 = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 dir =  glm::normalize(lp1 - lp0);
    glm::vec3 right = glm::normalize(glm::cross(dir, glm::vec3(0.0f, 0.0f, 1.0f)));
    glm::mat4 model =
            glm::perspective(glm::radians(45.0f), _window.width() / (float) _window.height(), 0.1f, 100.0f) *
            glm::lookAt(lp0,  lp0 + dir, glm::normalize(glm::cross(right, dir)));

    bool use_shadow = shadow && !points_mode;
    if (use_shadow) {
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, _fbo);
        glClear(GL_DEPTH_BUFFER_BIT);

        _shadow_shader->bind();
        _shadow_shader->uniform_mat4("proj", model);
        _plane->bind();
        _plane->draw(0, 4);
        _mesh->bind();
        _mesh->draw(0, _verts_n);

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    _shader->bind();
    _shader->uniform_mat4("proj", _proj);
    _shader->uniform_mat4("model", view);
    _shader->uniform3f("eye", eye);
    _shader->uniform1i("light", light);
    _shader->uniform1i("shadow", use_shadow);

    if(use_shadow) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, _shadow_map);
        _shader->uniform1i("map", 0);
        _shader->uniform_mat4("light_model", model);
    }

    if(ground_plane) {
        _plane->bind();
        _shader->uniform3f("color", glm::vec3(0.61f, 0.67f, 0.80f));
        _plane->draw(0, 4);
        _plane->unbind();
    }

    if(!points_mode) {
        _shader->uniform3f("color", glm::vec3(1.0f, 1.0f, 1.0f));
        _mesh->bind();
        _mesh->draw(0, _verts_n);
    }
    else {
        _point_shader->bind();
        _point_shader->uniform_mat4("proj", _proj);
        _point_shader->uniform_mat4("model", view);
        _point_shader->uniform1f("thresh", thresh);

        _points->bind();
        _points->draw(0, _points_n);
        _points->unbind();
    }

    glBindTexture(GL_TEXTURE_2D, 0);
}

void renderer::upload_points(sensors& fetcher)
{
    sensors::cloud_res_t& res = fetcher.cloud();

    res.mutex().lock();
    size_t to_load = res.value().points().size();

    if(to_load == 0) {
        res.mutex().unlock();
        return;
    }

    float* buf = new float[4 * to_load];
    size_t i = 0;
    for(const point_t& p : res.value().points()) {
        buf[4 * i + 0] = p.x;
        buf[4 * i + 1] = p.y;
        buf[4 * i + 2] = p.z;
        buf[4 * i + 3] = p.p;
        i++;
    }
    res.mutex().unlock();

    _points_n = to_load;

    _points->bind();
    _points->update(buf, to_load * 4);

    delete[] buf;
}

void renderer::upload_mesh(const std::vector<float> &data)
{
    _verts_n = data.size() / 6;

    _mesh->bind();
    _mesh->update(data.data(), data.size());
}

void renderer::clear_points()
{
    _points->bind();
    _points->update(nullptr, 0);
}

void renderer::init_shadows()
{
    glGenFramebuffers(1, &_fbo);

    glGenTextures(1, &_shadow_map);
    glBindTexture(GL_TEXTURE_2D, _shadow_map);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT,
                 _window.width(), _window.height(), 0,
                 GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glBindFramebuffer(GL_FRAMEBUFFER, _fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, _shadow_map, 0);

    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);

    GLenum Status = glCheckFramebufferStatus(GL_FRAMEBUFFER);

    if (Status != GL_FRAMEBUFFER_COMPLETE) {
        printf("FB error, status: 0x%x\n", Status);
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void renderer::resize() {
}
