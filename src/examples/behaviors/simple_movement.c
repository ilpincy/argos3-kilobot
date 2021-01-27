#include <kilolib.h>

void setup()
{
}

void loop()
{
    // Set the LED green.
    set_color(RGB(0, 1, 0));
    // Spinup the motors to overcome friction.
    spinup_motors();
    // Move straight for 2 seconds (2000 ms).
    set_motors(kilo_straight_left, kilo_straight_right);
    delay(2000);
}

int main()
{
    kilo_init();
    kilo_start(setup, loop);
    
    return 0;
}
