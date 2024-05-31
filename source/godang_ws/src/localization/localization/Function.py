import math
from PID import PIDController

class PositionController:
    def __init__(self):
        self.x, self.y, self.theta = 0, 0, 0
        self.PosX = PIDController(kP=0.1, kI=0.05, kD=0)
        self.PosY = PIDController(kP=0.1, kI=0.05, kD=0)
        self.PosZ = PIDController(kP=0.01, kI=0.001, kD=0)
        self.vx, self.vy, self.vz = 0, 0, 0
        self.reset = 0;
        
    def position_reset(self):
        self.reset = 1;

    def angular_difference(self, target, current):
        difference = target - current
        if difference < -180:
            difference += 360
        if difference > 180:
            difference -= 360
        return difference

    def go_to_position(self, target_x, target_y, target_z, pos_x, pos_y, pos_z):
        
        time = 0
        
        if self.reset == 1:
            pass
        else:
            
            if time == 0:
                total_distance = math.sqrt(target_x**2 + target_y**2)
                accel_decel_distance = total_distance / 3
                time += 1
            
            current_x, current_y, current_z = pos_x, pos_y, pos_z
            error_x = target_x - current_x
            error_y = target_y - current_y
            error_z = self.angular_difference(target_z, current_z)

            current_distance = math.sqrt(current_x**2 + current_y**2)

            error_magnitude = math.sqrt(error_x**2 + error_y**2)
            error_direction = math.atan2(error_y, error_x)

            if error_magnitude != 0:
                if current_distance < accel_decel_distance:
                    velocity_scale = error_magnitude * ((current_distance / accel_decel_distance)+0.2)
                elif current_distance > total_distance - accel_decel_distance:
                    velocity_scale = error_magnitude * (((total_distance - current_distance) / accel_decel_distance)+0.2)
                else:
                    velocity_scale = error_magnitude
            elif error_magnitude == 0:
                velocity_scale = 0

            vx = self.PosX.update(round(velocity_scale * math.cos(error_direction),1))
            vy = self.PosY.update(round(velocity_scale * math.sin(error_direction),1))
            vz = self.PosZ.update(error_z)

            if abs(error_x) <= 0.05 and abs(error_y) <= 0.05 and abs(error_z) <= 3:
                return [0.0,0.0,0.0,0.0]
            else:
                return [vx,vy,vz,0.0]