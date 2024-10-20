# Woof Band Firmware
### Dependencies
- The code was forked and edited from https://github.com/Riei-Joaquim/dog-step-counter
- The step counting algorithm with the mpu6050 was based on the available open-source implementation [Move Your Body](https://github.com/gabrielbvicari/myb)

### Operations
- Activate:
  1. Run server (see software repo)
  2. Change function call on main.cpp line 81 if measuring graphical data vs. not
  3. Edit data in /lib/mywifi/mywifi.h i.e. dog name, server url, wifi data
  4. Adjust step size in algorithm if needed
  5. Plug in Woof Band and select upload (->) at the bottom 