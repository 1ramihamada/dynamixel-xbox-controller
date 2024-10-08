# DYNAMIXEL XM430-W350 Xbox Controller

This project contains Python code for controlling a DYNAMIXEL XM430-W350 motor using an Xbox controller. The code utilizes the Dynamixel SDK for motor communication and the `inputs` library for Xbox controller inputs, allowing users to control the motor’s velocity and direction in real-time.

## Features
- Control motor rotation speed and direction using the D-pad on an Xbox controller.
- Set velocity limits and operating modes for the DYNAMIXEL motor.
- Monitor the present velocity and adjust it dynamically.
- Adjustable fixed velocity with D-pad up/down inputs.

## Requirements
- Python 3.x
- `dynamixel-sdk` library
- `inputs` library

## Setup
1. **Install Dependencies**:
   ```bash
   pip install dynamixel-sdk inputs

## Controls
- D-pad Left: Rotate counter-clockwise
- D-pad Right: Rotate clockwise
- D-pad Up: Increase speed
- D-pad Down: Decrease speed
- Release D-pad to stop (left/right)
- Press Start button to exit
