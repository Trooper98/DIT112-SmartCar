# DIT112-SmartCar
This is a course project that uses [@platisd](https://github.com/platisd) 's [smartcar_shield arduino library](https://github.com/platisd/smartcar_shield).

## What is does
Features include:
    Control vehicle in a tank like manner
    Avoid crashing into obstacles
    Parallel park vehicle
    Initiate Parking with voice command
    Stream live footage from the dash cam with very low buffer speeds (< 1 sec)

For a lil sneak peek of what you can do with this repository, checkout this [post](https://www.linkedin.com/feed/update/urn:li:activity:6410238729935417344) from my LinkedIn :)

## What do you need

Materials needed include:
    Toy car chasis
    L293D H-bridge
    GY-50 gyroscope module
    Two speed encoders
    Male & female pin headers (optionally get stackable arduino headers too)
    Five 2-pin screw terminals
    Pin jumper
    DIP16 socket
    9V or 12V battery pack (do not use a single 9V battery)
    Distance (where supported sensores include)
    Ultrasonic
        HC-SR04
        SRF08
    Infrared
        SHARP GP2D120
        SHARP GP2Y0A02
        SHARP GP2Y0A21
    Arduino
    RaspberryPi 3
    RaspberryPi camera
    Battery Pack (_for the RPi3_)
    A void recognition module (_I need to stress this enough. This module could understand voicers... but not words... security feature? meh_)
    A Microphone (_for the voice recognition module_)
    
## getting started
1. Clone this repository
2. Include the smartcar shiel into your Arduino IDE
3. Run our code on your car through a usb cord
4. When code finishes running successfully, start the car. 

## Aditional info
If you have difficulties setting up the vehicle, I suggest taking a look at the [smartcar_shield  library](https://github.com/platisd/smartcar_shield) repository that I mentioned above since it inlcude fantastic documentation!
    