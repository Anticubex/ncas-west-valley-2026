#include "main.h"
#include "pros/colors.hpp"
#include "pros/optical.hpp"
#include "pros/screen.hpp"

void do_mineral_detection(pros::Optical &sensor) {
    // Check yellow and green
    // Yellow
    if (sensor.get_hue() > 40 && sensor.get_hue() < 70 &&
        sensor.get_saturation() > 0.5) {
        pros::screen::set_eraser(pros::Color::yellow);
        pros::screen::erase();
    }
    // Green
    else if (sensor.get_hue() > 100 && sensor.get_hue() < 150 &&
             sensor.get_saturation() > 0.4) {
        pros::screen::set_eraser(pros::Color::green);
        pros::screen::erase();
    } else {
        pros::screen::set_eraser(pros::Color::black);
    }
}
