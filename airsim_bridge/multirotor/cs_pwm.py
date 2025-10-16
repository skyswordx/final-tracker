import airsim
import time

# Initialize the AirSim client for the multirotor
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
print("Connected to AirSim and API Control enabled.")

# Function to send motor PWMs
def send_motor_pwms(front_right_pwm, rear_left_pwm, front_left_pwm, rear_right_pwm, duration):
    """Send PWM commands to the drone's motors."""
    future = client.moveByMotorPWMsAsync(front_right_pwm, rear_left_pwm, front_left_pwm, rear_right_pwm, duration)
    return future

# Example usage: Set your desired PWM values and duration here
front_right_pwm = 0  # Example PWM values
rear_left_pwm = 0
front_left_pwm = 0
rear_right_pwm = 1
duration = 1  # Duration for how long to send the command

try:
    while True:
        # Send motor PWMs
        future = send_motor_pwms(front_right_pwm, rear_left_pwm, front_left_pwm, rear_right_pwm, duration)
        future.join()  # Wait for the command to complete
        print("PWM command sent. Looping...")

        # Adjust PWM values or add logic here as needed
        time.sleep(duration)  # Wait for the specified duration before sending the next command
except KeyboardInterrupt:
    print("Stopping PWM commands...")

# Cleanup
client.armDisarm(False)
client.enableApiControl(False)
print("Disconnected and cleaned up.")
