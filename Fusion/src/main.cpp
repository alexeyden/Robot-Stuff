#include <iostream>

#include "view/view_window.h"
#include "sensors/config.h"

int main(int argc, char* argv[]) {
    (void) argc;
    (void) argv;

    config::init("data/config.json");

    view_window window(800, 600);
    window.run_loop();

    return 0;
}
