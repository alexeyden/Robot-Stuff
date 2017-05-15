#ifndef VIEW_WINDOW_H
#define VIEW_WINDOW_H

#include <vector>
#include <memory>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "sensors/sensors.h"

#include "view.h"

class view_window
{
public:
    view_window(size_t width, size_t height);
    ~view_window();

    void run_loop();

    uint32_t width() const {
        return _width;
    }
    uint32_t height() const {
        return _height;
    }

    sensors& sensors_data() {
        return _sensors;
    }

    auto& views() {
        return _views;
    }

    GLFWwindow* glfw_window() {
        return _window;
    }

    float fps() const {
        return _ticks_per_sec;
    }

private:
    static void on_resize(GLFWwindow* win, int width, int height);
    static void on_mouse(GLFWwindow* win, double x, double y);
    static void on_key(GLFWwindow* win, int key, int code, int action, int mods);

    sensors _sensors;

    uint32_t _width;
    uint32_t _height;

    std::vector<std::shared_ptr<view>> _views;
    size_t _cur_view;
    GLFWwindow* _window;

    float _ticks_time;
    float _ticks_per_sec;
    uint32_t _ticks_count;
};

#endif // VIEW_WINDOW_H
