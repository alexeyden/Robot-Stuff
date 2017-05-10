#include "view_window.h"

#include <iostream>
#include <cstring>

#include "view_debug.h"

view_window::view_window(size_t width, size_t height)
{
    glfwInit();

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);

    _window = glfwCreateWindow(width, height, "View", nullptr, nullptr);
    glfwSetWindowUserPointer(_window, this);
    glfwSetFramebufferSizeCallback(_window, &on_resize);

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

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
//    glEnable(GL_DEPTH_TEST);
//    glDepthFunc(GL_LESS);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    _width = width;
    _height = height;

    _views.push_back(std::shared_ptr<view>(new view_debug(this)));

    std::cerr << "Window: starting fetcher thread.." << std::endl;
    _fetcher.start();
}

view_window::~view_window()
{
    std::cerr << "Window: waiting for fetcher thread to stop.." << std::endl;

    _fetcher.stop();

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

        for(auto& v : _views) {
            v->update(dt);
            v->draw();
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
