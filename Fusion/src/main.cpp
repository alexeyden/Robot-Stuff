#include <iostream>

#include "view/view_window.h"

int main(int argc, char* argv[]) {
    (void) argc;
    (void) argv;

    view_window window(800, 600);
    window.run_loop();

    return 0;
}
