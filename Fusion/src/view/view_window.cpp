#include "view_window.h"

#include <iostream>
#include <cstring>

#include "view_debug.h"
#include "main_view.h"

#include "sensors/crbuffer.h"

view_window::view_window(size_t width, size_t height)
{
    glfwInit();

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);

    glfwWindowHint(GLFW_RESIZABLE, false);
    _window = glfwCreateWindow(width, height, "View", nullptr, nullptr);
    glfwSetWindowSize(_window, 800, 600);
    glfwSetWindowUserPointer(_window, this);
    glfwSetCursorPosCallback(_window, &on_mouse);
    glfwSetKeyCallback(_window, &on_key);

    glfwMakeContextCurrent(_window);

    glewExperimental = true;
    glewInit();

    auto cb = [](GLenum src, GLenum type, GLuint id, GLenum sev, GLsizei len, const GLchar* msg, const void* param) {
        (void) src; (void) type;
        (void) id; (void) sev;
        (void) param;
        char* b = new char[len + 1];
        memcpy(b, msg, len);
        b[len] = 0;
        std::cerr << "OpenGL Log: " << b << std::endl;
        delete [] b;
    };

    glDebugMessageCallback(cb, nullptr);
    glEnable(GL_DEBUG_OUTPUT);
    glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);

    glClearColor(0.11, 0.17, 0.3, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glEnable(GL_PROGRAM_POINT_SIZE);

    _width = width;
    _height = height;

    _views.push_back(std::shared_ptr<view>(new view_debug(this)));
    _views.push_back(std::shared_ptr<view>(new main_view(this)));
    _cur_view = 0;

    _ticks_count = 0;
    _ticks_per_sec = 0.0f;
    _ticks_time = 0.0f;

    std::cerr << "Window: starting fetcher thread.." << std::endl;
    _sensors.start();
}

view_window::~view_window()
{
    std::cerr << "Window: waiting for fetcher thread to stop.." << std::endl;

    _sensors.stop();

    glfwDestroyWindow(_window);
    glfwTerminate();
}

void view_window::run_loop()
{
    double t0 = glfwGetTime();

    while(!glfwWindowShouldClose(_window)) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        double dt = glfwGetTime() - t0;
        t0 = glfwGetTime();

        _views[_cur_view]->update(dt);
        _views[_cur_view]->draw();

        _ticks_time += dt;
        _ticks_count += 1;

        if(_ticks_count > 10) {
            _ticks_per_sec = _ticks_count / _ticks_time;
            _ticks_time = 0.0f;
            _ticks_count = 0;
        }

        glfwPollEvents();
        glfwSwapBuffers(_window);
    }
}

void view_window::on_resize(GLFWwindow *win, int width, int height)
{
    view_window* self = (view_window*) glfwGetWindowUserPointer(win);

    for(auto& v : self->_views)
        v->resize(width, height);

    glViewport(0, 0, width, height);

    self->_width = width;
    self->_height = height;
}

void view_window::on_mouse(GLFWwindow *win, double x, double y)
{
    view_window* self = (view_window*) glfwGetWindowUserPointer(win);
    self->_views[self->_cur_view]->on_mouse(x, y);
}

void view_window::on_key(GLFWwindow *win, int key, int code, int action, int mods)
{
    (void) mods;
    (void) code;
    view_window* self = (view_window*) glfwGetWindowUserPointer(win);
    if(key == GLFW_KEY_1 && action == GLFW_PRESS)
        self->_cur_view = 0;
    if(key == GLFW_KEY_2 && action == GLFW_PRESS)
        self->_cur_view = 1;

    self->_views[self->_cur_view]->on_key(key, action == GLFW_RELEASE);
}
