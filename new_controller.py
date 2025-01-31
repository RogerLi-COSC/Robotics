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


#controller = Controller('COM9')
ik = InverseKinematics()

def test_controller_stdout():
    # Create instances of the necessary components
    controller = Controller(None)  # Remove the serial dependency by passing None
    controller.ser = None  # Disable serial communication

    # --- Test 1: Move to a valid target position ---
    print("\n[Test 1] Moving to a valid target position")
    valid_target = Vec(5, controller.inner_arm + controller.outer_arm - 5)
    try:
        controller.move_to_xy(valid_target)
        print("Test 1 passed: Successfully moved to valid position")
    except Exception as e:
        print(f"Test 1 failed: {e}")

    # --- Test 2: Move to an out-of-reach position ---
    print("\n[Test 2] Attempting to move to an out-of-reach position")
    out_of_reach_target = Vec(0, controller.inner_arm + controller.outer_arm + 10)
    try:
        controller.move_to_xy(out_of_reach_target)
        print("Test 2 failed: No error raised for out-of-reach position")
    except Exception as e:
        print(f"Test 2 passed: Correctly handled out-of-reach position ({e})")

    # --- Test 3: Perform round-trip position verification ---
    print("\n[Test 3] Performing round-trip test")
    original_target = Vec(6, 6)
    try:
        # Step 1: Calculate angles
        angles = controller.ik.calculateAngles(original_target, controller.inner_arm, controller.outer_arm)
        print(f"Debug: Calculated angles = {angles}")

        # Step 2: Convert angles back to position using manual forward kinematics
        q1_rad = math.radians(angles.x)
        q2_rad = math.radians(angles.y - 90)
        x_test = controller.inner_arm * math.cos(q1_rad) + controller.outer_arm * math.cos(q1_rad + q2_rad)
        y_test = controller.inner_arm * math.sin(q1_rad) + controller.outer_arm * math.sin(q1_rad + q2_rad)
        
        print(f"Debug: Forward kinematics result = (x: {x_test}, y: {y_test})")

        # Step 3: Compare positions
        error_margin = 0.1
        if abs(original_target.x - x_test) < error_margin and abs(original_target.y - y_test) < error_margin:
            print("Test 3 passed: Round-trip accuracy is within acceptable margin")
        else:
            print(f"Test 3 failed: Round-trip error is too large (x: {x_test}, y: {y_test})")
        print(f"Debug: Original target = (x: {original_target.x}, y: {original_target.y})")
    except Exception as e:
        print(f"Test 3 failed: Error occurred during round-trip test ({e})")

    # --- Test 4: Basic pen control simulation ---
    print("\n[Test 4] Simulating pen control")
    try:
        controller.set_pen(True)
        print("Pen set to down position")
        controller.set_pen(False)
        print("Pen set to up position")
        print("Test 4 passed: Pen control simulation completed")
    except Exception as e:
        print(f"Test 4 failed: {e}")

# Run the tests
test_controller_stdout()



# controller.move_to_angle(Vec(90,45))
#time.sleep(2)
#for i in range(5):
    #try:
    #    controller.move_to_xy(Vec(0.005, ik.INNER_LEN+ik.OUTER_LEN-i-1))
    #except:
    #    print("can't go to:")
    #    Vec(0.005, ik.INNER_LEN+ik.OUTER_LEN-i-1).print()
    #print("---------------------------------")

