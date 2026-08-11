This is an upgrade to the Entosieve, an automated bug-sorter. The original used custom electronics with an Arduino and analog controls. This updates the electronics to an ESP32 with a built-in stepper motor board that connects via Wifi to a local network and creates a Web app for fully digital control. Because it uses M5Stack hardware, it also has a screen and buttons for local/manual control if needed. 

The advantage of digital control is that it can be monitored and controlled remotely and all settings and run metrics can by recorded as a part of fully digital pipeline in a lab. 

The Web app controls:
- homing
- top and bottom position
- speed
- duration of sieving cycle
- saving settings

Electronics BOM:

- [M5Stack core](https://shop.m5stack.com/products/esp32-basic-core-lot-development-kit-v2-7)
- [M5Stack Stepper module](https://shop.m5stack.com/products/stepmotor-driver-module-v1-1-hr8825)

