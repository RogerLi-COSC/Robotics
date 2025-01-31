import serial
import time
import math

class Vec:
    x = 0
    y = 0

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def print(self):
        print(f"x: {self.x}, y: {self.y}")

    def __repr__(self):
        return f"Vec(x={self.x}, y={self.y})"


BOTTOM_LEFT = Vec(424,443)

class Brachiograph:
    #max right
    SHOULDER_MIN_DUTY = 75
    SHOULDER_MIN_ANGLE = 47
    SHOULDER_MAX_DUTY = 444
    SHOULDER_MAX_ANGLE = 210
    SHOULDER_90 = 176

    #max left
    ELBOW_MIN_DUTY = 470
    ELBOW_MAX_ANGLE = 150
    #max right
    ELBOW_MIN_ANGLE = -25
    ELBOW_MAX_DUTY = 75
    ELBOW_90 = 170

    def angle_to_pulse(self, angle, max_angle, min_angle, max_duty, min_duty, duty_90):
        # Handle inverted duty cycles
        if max_duty < min_duty:
            max_duty, min_duty = min_duty, max_duty  # Swap the values

        delta_duty = max_duty - min_duty
        delta_angle = max_angle - min_angle
        conversion_factor = delta_duty / delta_angle

        offset = (max_duty + min_duty) / 2 - (90 * conversion_factor)

        # Debug output
        print(f"Debug: angle = {angle}, conversion_factor = {conversion_factor}, offset = {offset}")

        return (angle * conversion_factor) + offset

    def angles_to_pulse(self, vec):
        a_shoulder = vec.x
        p_shoulder = self.angle_to_pulse(
            a_shoulder, self.SHOULDER_MAX_ANGLE, self.SHOULDER_MIN_ANGLE,
            self.SHOULDER_MAX_DUTY, self.SHOULDER_MIN_DUTY, self.SHOULDER_90
        )
        print(f"Debug: Shoulder pulse = {p_shoulder}, Expected Range = ({self.SHOULDER_MIN_DUTY}, {self.SHOULDER_MAX_DUTY})")

        a_elbow = vec.y
        p_elbow = self.angle_to_pulse(
            a_elbow, self.ELBOW_MAX_ANGLE, self.ELBOW_MIN_ANGLE,
            self.ELBOW_MAX_DUTY, self.ELBOW_MIN_DUTY, self.ELBOW_90
        )
        print(f"Debug: Elbow pulse = {p_elbow}, Expected Range = ({self.ELBOW_MIN_DUTY}, {self.ELBOW_MAX_DUTY})")

        # Handle inverted ranges
        if self.SHOULDER_MIN_DUTY > self.SHOULDER_MAX_DUTY:
            if not (self.SHOULDER_MAX_DUTY <= p_shoulder <= self.SHOULDER_MIN_DUTY):
                print("Shoulder pulse out of bounds")
                raise ValueError("Shoulder pulse out of bounds")
        else:
            if not (self.SHOULDER_MIN_DUTY <= p_shoulder <= self.SHOULDER_MAX_DUTY):
                print("Shoulder pulse out of bounds")
                raise ValueError("Shoulder pulse out of bounds")

        if self.ELBOW_MIN_DUTY > self.ELBOW_MAX_DUTY:
            if not (self.ELBOW_MAX_DUTY <= p_elbow <= self.ELBOW_MIN_DUTY):
                print("Elbow pulse out of bounds")
                raise ValueError("Elbow pulse out of bounds")
        else:
            if not (self.ELBOW_MIN_DUTY <= p_elbow <= self.ELBOW_MAX_DUTY):
                print("Elbow pulse out of bounds")
                raise ValueError("Elbow pulse out of bounds")

        return Vec(p_shoulder, p_elbow)


class InverseKinematics:
    INNER_LEN = 8.6
    OUTER_LEN = 9.7
    def q2(self, x, y, inner_len, outer_len):
        return math.acos(
            (x**2 + y**2 - inner_len**2 - outer_len**2)/
            (2*inner_len*outer_len)
        )
    
    def q1(self, x, y, inner_len, outer_len, q2):
        return (
            math.atan(y/x)-
            math.atan(
                (outer_len * math.sin(q2))/
                (inner_len + outer_len* math.cos(q2))
            )
        )

    def calculateAngles(self, target_pos, link1len, link2len):
        try:
            q2 = self.q2(target_pos.x, target_pos.y, link1len, link2len)
            q1 = self.q1(target_pos.x, target_pos.y, link1len, link2len, q2)

            # Convert to degrees
            q1_deg = math.degrees(q1)
            q2_deg = math.degrees(q2)

            print(f"Debug: q1 (radians) = {q1}, q2 (radians) = {q2}")
            print(f"Debug: q1 (degrees) = {q1_deg}, q2 (degrees) = {q2_deg}")

            return Vec(q1_deg, 90 + q2_deg)
        except Exception as e:
            print(f"Error in calculateAngles(): {e}")
            raise

class Controller:
    curr_pos = BOTTOM_LEFT
    pen_set = False
    inner_arm = 8.5725
    outer_arm = 10.4

    brach = Brachiograph()
    ik = InverseKinematics()

    def __init__(self, port):
        self.port = port
        if port is not None:
            self.ser = serial.Serial(port, 115200, timeout=1)
            print(self.ser.readline().decode('utf-8').strip())
            self.move_to_absolute(BOTTOM_LEFT)
        else:
            self.ser = None  # No serial communication when port is None

    def create_str(vec, pen_set):
        set = 1
        if pen_set:
            set = 2
        return "{" + str(vec.x) + "," + str(vec.y) + "," + str(set) + "}\n"

    def move_to_absolute(self, vec):
        self.curr_pos = vec
        string = Controller.create_str(vec, self.pen_set)
        if self.ser:
            self.ser.write(bytes(string, 'utf-8'))
            # Wait for a response from the robot if serial is enabled
            for i in range(5):
                result = self.ser.readline().decode('utf-8').strip()
                if result != "":
                    print(result)
                    break
            time.sleep(0.8)
        else:
            # Simulate the behavior without serial
            print(f"Simulated move to absolute position: {vec}")

    def set_pen(self, is_down):
        self.pen_set = is_down
        self.move_to_absolute(self.curr_pos)

    def move_to_xy(self, target_pos):
        print("Moving to: x:" + str(target_pos.x) + ", y:" + str(target_pos.y))
        angles = self.ik.calculateAngles(target_pos, self.ik.INNER_LEN, self.ik.OUTER_LEN)
        print("Angles:")
        angles.print()
        print("Pulse:")
        pulse = self.brach.angles_to_pulse(angles)
        pulse.print()
        self.move_to_absolute(pulse)

    def move_to_angle(self, vec):
        pulse = self.brach.angles_to_pulse(vec)
        self.move_to_absolute(pulse)


ik = InverseKinematics()

def test_controller_real_with_points():
    # Create the controller with real serial communication (e.g., COM9 or other port)
    controller = Controller('COM9')  # Replace 'COM9' with the correct port for your setup

    points_to_draw = [Vec(5, 5), Vec(6, 6), Vec(7, 7)]

    # --- Draw points ---
    print("\n[Test: Drawing Points]")
    controller.set_pen(False)  # Raise the pen initially
    for point in points_to_draw:
        try:
            controller.move_to_xy(point)
            controller.set_pen(True)   # Lower the pen to draw
            controller.set_pen(False)  # Raise the pen after drawing each point
            print(f"Successfully moved to point: {point}")
        except Exception as e:
            print(f"Failed to move to point: {point} ({e})")

# Run the test
test_controller_real_with_points()
