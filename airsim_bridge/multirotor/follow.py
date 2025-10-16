import airsim
import time
import pprint


class PIDController:
    def __init__(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.previous_error = 0
        self.integral = 0
        
    def reset(self):
        self.previous_error = 0
        self.integral = 0
        
    def calculate(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output

pid_x = PIDController(0.5, 0.01, 0.05)
pid_y = PIDController(0.5, 0.01, 0.05)
pid_z = PIDController(0.5, 0.01, 0.05)

client = airsim.MultirotorClient(ip="")
client.confirmConnection()
client.enableApiControl(True)

print("Arming the drone...")
client.armDisarm(True)

target_height_above_person = 50.0 / 100.0  # 50cm above the person's head

while True:
    person1_pose = client.simGetObjectPose("Person1")
    drone_pose = client.simGetVehiclePose()

    error_x = person1_pose.position.x_val - drone_pose.position.x_val
    error_y = person1_pose.position.y_val - drone_pose.position.y_val

    target_z = person1_pose.position.z_val - target_height_above_person
    error_z = target_z - drone_pose.position.z_val

    control_signal_x = pid_x.calculate(error_x, 0.02)
    control_signal_y = pid_y.calculate(error_y, 0.02)
    control_signal_z = pid_z.calculate(error_z, 0.02)
    
    client.moveByVelocityAsync(control_signal_x, control_signal_y, control_signal_z, 1, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 0))
    
    time.sleep(0.02)
    