#ifndef VIEW_WINDOW_H
#define VIEW_WINDOW_H

#include <vector>
#include <memory>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "sensors/fetcher.h"

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

    fetcher& data_fetcher() {
        return _fetcher;
    }

private:
    static void on_resize(GLFWwindow* win, int width, int height);

    fetcher _fetcher;

    uint32_t _width;
    uint32_t _height;

    std::vector<std::shared_ptr<view>> _views;
    GLFWwindow* _window;
};

#endif // VIEW_WINDOW_H
