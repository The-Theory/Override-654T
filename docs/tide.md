# Tide

Controller bindings for 654T Tsunami. Declare what each button does once,
then poll everything with a single `update()` call per opcontrol loop.

```cpp
#include "tide/control.hpp"
using namespace tide::btn;

void opcontrol() {
    tide::Control ctl(controller);
    ctl.bidir(intakeMotor, R)      // R1 forward, R2 reverse
       .bidir(winch, -L)           // L2 forward, L1 reverse
       .toggle(clawPivot, A);

    while (true) {
        ctl.update();
        chassis.curvature(ctl.axis(LY), ctl.axis(RX));
        pros::delay(25);
    }
}
```