#ifndef VIEW_H
#define VIEW_H

class view_window;

class view
{
public:
    view(view_window* parent) : _window(parent) {}
    virtual ~view() {}

    virtual void update(float dt) = 0;
    virtual void draw() = 0;
    virtual void resize(int w, int h) {
        (void) w;
        (void) h;
    }
    virtual void on_mouse(float x, float y) {
        (void) x;
        (void) y;
    }
    virtual void on_key(int key, bool released) {
        (void) key;
        (void) released;
    }

protected:
    view_window* _window;
};

#endif // VIEW_H
