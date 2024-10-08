from dynamixel_sdk import *  # Uses Dynamixel SDK library
from inputs import get_gamepad
import threading
import time

# Control table addresses
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_VELOCITY = 128
ADDR_OPERATING_MODE = 11
ADDR_VELOCITY_LIMIT = 44  # Address for Velocity Limit

# Protocol version
PROTOCOL_VERSION = 2.0

# Default settings
DXL_ID = 1  # Dynamixel ID
BAUDRATE = 57600
DEVICENAME = '/dev/ttyUSB0'  # Adjust if necessary

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
VELOCITY_CONTROL_MODE = 1  # Operating mode for Velocity Control

class DynamixelController:
    def __init__(self):
        self.lock = threading.Lock()
        try:
            # Initialize PortHandler and PacketHandler
            self.portHandler = PortHandler(DEVICENAME)
            self.packetHandler = PacketHandler(PROTOCOL_VERSION)

            # Open port
            if self.portHandler.openPort():
                print("Succeeded to open the port")
            else:
                raise Exception("Failed to open the port")

            # Set port baudrate
            if self.portHandler.setBaudRate(BAUDRATE):
                print("Succeeded to change the baudrate")
            else:
                raise Exception("Failed to change baudrate")

            # Disable torque before changing settings
            self.disable_torque()
            print("Dynamixel torque disabled")

            # Set operating mode to Velocity Control Mode
            self.set_operating_mode(VELOCITY_CONTROL_MODE)
            print("Operating mode set to Velocity Control Mode")

            # Set Velocity Limit to maximum allowable value
            MAX_VELOCITY_LIMIT = 1023  # As per your motor's specifications

            # Set the velocity limit in the motor's control table
            self.set_velocity_limit(MAX_VELOCITY_LIMIT)

            # Read Velocity Limit to confirm
            dxl_velocity_limit = self.get_velocity_limit()
            print(f"Velocity Limit set to: {dxl_velocity_limit}")

            # Set MAX_VELOCITY to the velocity limit
            self.MAX_VELOCITY = dxl_velocity_limit

            # Re-enable torque
            self.enable_torque()
            print("Dynamixel torque enabled")

        except Exception as e:
            print(f"Initialization Error: {e}")
            self.cleanup()
            raise

    def set_velocity_limit(self, limit):
        with self.lock:
            # Write velocity limit
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
                self.portHandler, DXL_ID, ADDR_VELOCITY_LIMIT, int(limit))
        if dxl_comm_result != COMM_SUCCESS:
            raise Exception(f"Failed to set velocity limit: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            raise Exception(f"Dynamixel error while setting velocity limit: {self.packetHandler.getRxPacketError(dxl_error)}")

    def get_velocity_limit(self):
        with self.lock:
            dxl_velocity_limit, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
                self.portHandler, DXL_ID, ADDR_VELOCITY_LIMIT)
        if dxl_comm_result != COMM_SUCCESS:
            raise Exception(f"Failed to read velocity limit: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            raise Exception(f"Dynamixel error while reading velocity limit: {self.packetHandler.getRxPacketError(dxl_error)}")
        else:
            return dxl_velocity_limit

    def enable_torque(self):
        with self.lock:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                self.portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            raise Exception(f"Failed to enable torque: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            raise Exception(f"Dynamixel error while enabling torque: {self.packetHandler.getRxPacketError(dxl_error)}")

    def disable_torque(self):
        with self.lock:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                self.portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            raise Exception(f"Failed to disable torque: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            raise Exception(f"Dynamixel error while disabling torque: {self.packetHandler.getRxPacketError(dxl_error)}")

    def set_operating_mode(self, mode):
        with self.lock:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                self.portHandler, DXL_ID, ADDR_OPERATING_MODE, mode)
        if dxl_comm_result != COMM_SUCCESS:
            raise Exception(f"Failed to set operating mode: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            raise Exception(f"Dynamixel error while setting operating mode: {self.packetHandler.getRxPacketError(dxl_error)}")

    def cleanup(self):
        if hasattr(self, 'portHandler') and self.portHandler.is_open:
            with self.lock:
                self.portHandler.closePort()

    def set_goal_velocity(self, goal_velocity):
        with self.lock:
            # Enforce limits
            goal_velocity = max(min(goal_velocity, self.MAX_VELOCITY), -self.MAX_VELOCITY)

            # Convert signed integer to unsigned 32-bit integer
            if goal_velocity < 0:
                goal_velocity = goal_velocity + (1 << 32)

            # Write goal velocity
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
                self.portHandler, DXL_ID, ADDR_GOAL_VELOCITY, int(goal_velocity))
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Failed to write goal velocity: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"Dynamixel error while writing goal velocity: {self.packetHandler.getRxPacketError(dxl_error)}")
        else:
            print(f"Setting goal velocity: {goal_velocity}")

    def get_present_velocity(self):
        with self.lock:
            dxl_present_velocity, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
                self.portHandler, DXL_ID, ADDR_PRESENT_VELOCITY)
            if dxl_comm_result != COMM_SUCCESS:
                print(f"Failed to read present velocity: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                return None
            elif dxl_error != 0:
                print(f"Dynamixel error while reading present velocity: {self.packetHandler.getRxPacketError(dxl_error)}")
                return None

            # Convert unsigned 32-bit integer to signed integer
            if dxl_present_velocity > 0x7FFFFFFF:
                dxl_present_velocity -= (1 << 32)

        return dxl_present_velocity

    def stop(self):
        try:
            # Stop the motor by setting goal velocity to zero
            self.set_goal_velocity(0)
            self.disable_torque()
            if self.portHandler.is_open:
                with self.lock:
                    self.portHandler.closePort()
        except Exception as e:
            print(f"Error during stop: {e}")

    def __del__(self):
        self.stop()

class XboxControllerHandler:
    def __init__(self, dynamixel_controller):
        self.dynamixel_controller = dynamixel_controller
        self.moving_velocity = 0
        self.max_velocity = self.dynamixel_controller.MAX_VELOCITY  # Maximum velocity value from the controller
        self.fixed_velocity = 500  
        self.running = True

    def gamepad_loop(self):
        while self.running:
            events = get_gamepad()
            for event in events:
                # Handle D-pad Left/Right
                if event.ev_type == 'Absolute' and event.code == 'ABS_HAT0X':
                    # D-pad left/right
                    if event.state == -1:
                        # D-pad left pressed
                        self.moving_velocity = self.fixed_velocity
                    elif event.state == 1:
                        # D-pad right pressed
                        self.moving_velocity = -self.fixed_velocity
                    elif event.state == 0:
                        # D-pad released
                        self.moving_velocity = 0

                # Handle D-pad Up/Down
                elif event.ev_type == 'Absolute' and event.code == 'ABS_HAT0Y':
                    # D-pad up/down
                    if event.state == -1:
                        # D-pad up pressed, increase fixed_velocity
                        self.fixed_velocity += 100  # Increase by 100
                        if self.fixed_velocity > self.max_velocity:
                            self.fixed_velocity = self.max_velocity
                        print(f"Fixed velocity increased to: {self.fixed_velocity}")
                    elif event.state == 1:
                        # D-pad down pressed, decrease fixed_velocity
                        self.fixed_velocity -= 100  # Decrease by 100
                        if self.fixed_velocity < 0:
                            self.fixed_velocity = 0
                        print(f"Fixed velocity decreased to: {self.fixed_velocity}")

                elif event.ev_type == 'Key' and event.code == 'BTN_START' and event.state == 1:
                    # Start button pressed, exit
                    print("Start button pressed, exiting...")
                    self.running = False
                    break

            # Set the motor's goal velocity
            self.dynamixel_controller.set_goal_velocity(self.moving_velocity)
            time.sleep(0.05)  # Adjust the update rate as needed

    def stop(self):
        self.running = False
        # Ensure the motor is stopped when exiting
        self.dynamixel_controller.set_goal_velocity(0)

def main():
    controller = None
    xbox_handler = None
    try:
        controller = DynamixelController()
        xbox_handler = XboxControllerHandler(controller)

        print("Use the D-pad to control the Dynamixel motor:")
        print("D-pad Left: Rotate counter-clockwise")
        print("D-pad Right: Rotate clockwise")
        print("D-pad Up: Increase speed")
        print("D-pad Down: Decrease speed")
        print("Release D-pad to stop (left/right)")
        print("Press Start button to exit")

        # Start the gamepad input thread
        gamepad_thread = threading.Thread(target=xbox_handler.gamepad_loop)
        gamepad_thread.start()

        # Keep the main thread alive until the gamepad loop ends
        while xbox_handler.running:
            time.sleep(1)

    except KeyboardInterrupt:
        print("KeyboardInterrupt received, stopping...")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if xbox_handler:
            xbox_handler.stop()
        if controller:
            controller.stop()

if __name__ == '__main__':
    main()
