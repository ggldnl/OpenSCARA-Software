# OpenSCARA Software

This repository contains the software for my SCARA.

For a complete overview of the project, refer to the [main OpenSCARA repository](https://github.com/ggldnl/OpenSCARA). 

## 📝 Notes

1. I used an Arduino Uno. If you plan to use other boards, you should change the config file. For portability reasons (I created a python wrapper with cython for the arduino code in the [simulation repository](https://github.com/ggldnl/OpenSCARA-Simulation)), no explicit reference to the Arduino.h library is used in the config file (e.g. pin for end effector is 17 and corresponds to A3).

2. You can easily adjust the parameters regarding the kinematic structure, in case you modify the 3d models, without having to edit the code - just use the config.

## 🤝 Contribution

Feel free to contribute by opening issues or submitting pull requests. For further information, check out the [main Hexapod repository](https://github.com/ggldnl/Hexapod). Give a ⭐️ to this project if you liked the content.