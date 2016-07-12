#ifndef REEDS_SHEPP_DEFINITIONS_HPP
#define REEDS_SHEPP_DEFINITIONS_HPP

namespace astar {

    // define the steering values
    enum Steer {RSTurnLeft, RSStraight, RSTurnRight};

    const static unsigned int NumSteering = 3;

    // define the Gears
    enum Gear {ForwardGear, BackwardGear};

    // define the sizes
    const static unsigned int NumGears = 2;

}

#endif
