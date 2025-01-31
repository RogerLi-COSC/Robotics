
import serial
import time
import math

class Vec:
  x=0
  y=0
  def print(self):
      print("x: "+str(self.x)+", y: "+str(self.y))
  def __init__(self, x, y):
    self.x = x
    self.y = y

BOTTOM_LEFT = Vec(424,443)
BOTTOM_RIGHT = Vec(214,415)
TOP_LEFT = Vec(262,158)
TOP_RIGHT = Vec(124,171)

SERVO_MIN = 75
SERVO_ELBOW_MAX = 470
SERVO_SHOULDER_MAX = 444

def angle_to_pulse(angle, max_duty, min_duty):
    max_angle = 180
    min_angle = 0
    # return ((
    #     (max_duty-min_duty*angle-min_angle)/(max_angle-min_angle)
    # ) + min_duty)
    total_duty = max_duty - min_duty
    pulse_p_angle = total_duty/max_angle
    return (angle * pulse_p_angle) + min_duty


def angles_to_pulse(vec):
    angle_shoulder = vec.x
    pulse_shoulder = angle_to_pulse(
        angle_shoulder, 
        SERVO_SHOULDER_MAX,
        SERVO_MIN
    )
    angle_elbow = vec.y
    pulse_elbow = angle_to_pulse(
        angle_elbow, 
        SERVO_ELBOW_MAX,
        SERVO_MIN
    )
    return Vec(pulse_shoulder, pulse_elbow)

ELBOW_0 = 170
SHOULDER_0 = 175

#do before converting angles to pulse
def apply_ofsets(vec):
    shoulder_90 = angle_to_pulse(90, SERVO_SHOULDER_MAX, SERVO_MIN)
    pulse_shoulder_ofset = shoulder_90 - SHOULDER_0 

    elbow_90 = angle_to_pulse(90, SERVO_ELBOW_MAX, SERVO_MIN)
    pulse_elbow_ofset = elbow_90 - ELBOW_0 
    return Vec(pulse_shoulder_ofset + vec.x, pulse_elbow_ofset+vec.y)


class Controller:
    curr_pos = BOTTOM_LEFT
    pen_set = False
    inner_arm = 8.5725
    outer_arm = 10.4

    def create_str(vec, pen_set):
        set = 1
        if pen_set:
            set=2
        return "{"+ str(vec.x) +","+str(vec.y)+","+str(set)+"}\n"

    def move_to_absolute(self, vec):
        self.curr_pos = vec
        string = Controller.create_str(vec, self.pen_set)
        self.ser.write(bytes(string, 'utf-8'))
        #await confirmation from the robot
        for i in range(5):
            result = self.ser.readline().decode('utf-8').strip()
            if result != "":
                print(result)
                break  
        time.sleep(0.8)

    # def uv_to_angles(self, uv_x, uv_y):
    #     real_x = (uv_x * 12.7) # - 4.7625
    #     print("real x: "+str(real_x))
    #     real_y = (uv_y * 10.16) + 7.62
    #     print("real y: "+str(real_y))
    #     hypotenuse = math.sqrt(real_x ** 2 + real_y ** 2)
    #     hypotenuse_angle = math.asin(real_x/hypotenuse)

    #     inner_angle = math.acos(
    #         (hypotenuse ** 2 + self.inner_arm ** 2 - self.outer_arm ** 2) 
    #         / 
    #         (2 * hypotenuse * self.inner_arm)
    #     )
    #     shoulder_motor_angle = hypotenuse_angle - inner_angle

    #     outer_angle = math.acos(
    #         (self.inner_arm ** 2 + self.outer_arm ** 2 - hypotenuse ** 2) / (2 * self.inner_arm * self.outer_arm)
    #     )

    #     elbow_motor_angle = math.pi - outer_angle
    #     return Vec(math.degrees(shoulder_motor_angle), math.degrees(elbow_motor_angle))

    # def move_to_uv(self, uv_x, uv_y):
    #     print("moving to x: "+str(uv_x)+", y: "+str(uv_y))
    #     angles = self.uv_to_angles(uv_x,uv_y)
    #     angles.print()
    #     corrected_angles = angles_to_pulse(angles)
    #     corrected_angles.print()
    #     self.move_to_absolute(corrected_angles)

    def xy_to_angles(self, x, y):
        hypotenuse = math.sqrt(x ** 2 + y ** 2)
        if hypotenuse == 0:
            raise ValueError("Cannot calculate angles for x=0 and y=0 (origin).")
    
        hypotenuse_angle = math.asin(max(-1, min(1, x / hypotenuse)))
        inner_angle = math.acos(
        max(-1, min(1, (hypotenuse ** 2 + self.inner_arm ** 2 - self.outer_arm ** 2) / (2 * hypotenuse * self.inner_arm)))
        )
        shoulder_motor_angle = hypotenuse_angle - inner_angle

        outer_angle = math.acos(
        max(-1, min(1, (self.inner_arm ** 2 + self.outer_arm ** 2 - hypotenuse ** 2) / (2 * self.inner_arm * self.outer_arm)))
        )
        elbow_motor_angle = math.pi - outer_angle

        return Vec(math.degrees(shoulder_motor_angle), math.degrees(elbow_motor_angle))


    #main arm ofsets, written first
    #full to left
    INNER_MAX = 440
    #full to right
    INNER_MIN = 76
    INNER_0 = 175

    #little arm with pen, written second
    #full to right
    OUTER_MAX = 465
    #full to left
    OUTER_MIN = 76
    OUTER_0 = 170

    #the brach seems to be about double 
    def angles_to_brach_angles(self, vec):
        inner_angle = ((self.INNER_0/2)+vec.x)*2
        outer_angle = ((self.OUTER_0/2)+vec.y)*2

        return Vec(inner_angle, outer_angle)

    def move_to_xy(self, x, y):
        print("moving to x: "+str(x)+", y: "+str(y))
        angles = self.xy_to_angles(x,y)
        angles.print()
        corrected_angles = angles_to_pulse(angles)
        corrected_angles.print()
        ofset_angles = apply_ofsets(corrected_angles)
        ofset_angles.print()
        self.move_to_absolute(ofset_angles)

    def set_pen(self, is_down):
        self.pen_set = is_down
        self.move_to_absolute(self.curr_pos)
        time.sleep(0.8)

    def __init__(self, port):
        self.port = port
        self.ser = serial.Serial(port, 115200, timeout=1)
        print(self.ser.readline().decode('utf-8').strip())
        self.move_to_absolute(BOTTOM_LEFT)

controller = Controller('COM4')
time.sleep(2)
# controller.move_to_absolute(Vec(Controller.INNER_0, Controller.OUTER_0))
# time.sleep(2)
# controller.move_to_absolute(Vec(Controller.INNER_MIN, Controller.OUTER_MIN))
# time.sleep(2)

# controller.move_to_xy(2,2)


# controller.move_to_xy(10,1)
controller.set_pen(True)
# controller.move_to_xy(10,2)
# controller.move_to_xy(10,3)
# controller.move_to_xy(10,4)
# controller.move_to_xy(10,6)
# controller.move_to_xy(10,8)
# controller.move_to_xy(10,9)
# controller.move_to_xy(10,10)

# controller.move_to_xy(6,6)
# controller.move_to_xy(0,18)
# controller.move_to_xy(0,6)

controller.move_to_absolute(TOP_LEFT)
controller.move_to_absolute(TOP_RIGHT)


# controller.move_to_absolute(TOP_LEFT)
# controller.move_to_absolute(TOP_RIGHT)
# controller.move_to_absolute(BOTTOM_RIGHT)
# controller.move_to_absolute(BOTTOM_LEFT)
# controller.move_to_xy(3,3)
# controller.move_to_xy(4,4)
# controller.move_to_xy(5,5)
# controller.move_to_xy(7,7)
# controller.move_to_xy(9,9)
# controller.move_to_xy(11,11)
# time.sleep(2)
controller.set_pen(False)

# controller.move_to_absolute(BOTTOM_LEFT)