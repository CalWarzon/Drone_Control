import time


class SafetyManager:

    def __init__(self):

        self.armed = False

        self.last_controller_time = time.perf_counter()

        self.last_loop_time = time.perf_counter()

        self.failsafe = False

        self.max_angle = 70

        self.controller_timeout = 0.5

        self.loop_timeout = 0.1

        self.arm_hold_time = 1.0

        self.arm_start = None

    # ----------------------------------------
    # ARMING LOGIC
    # ----------------------------------------

    def UpdateArming(self, controller):

        """
        Arm:
            LT + RT + GUIDE held

        Disarm:
            BACK button
        """

        if controller["back"]:

            self.armed = False

            print("DISARMED")

            return

        arm_condition = (
            controller["start"] and
            controller["lt"] > 0.9 and
            controller["rt"] > 0.9
        )

        if arm_condition:

            if self.arm_start is None:
                self.arm_start = time.perf_counter()

            elapsed = (
                time.perf_counter() -
                self.arm_start
            )

            if elapsed > self.arm_hold_time:

                self.armed = True

                print("ARMED")

        else:
            self.arm_start = None

    # ----------------------------------------
    # CONTROLLER WATCHDOG
    # ----------------------------------------

    def UpdateController(self):

        self.last_controller_time = (
            time.perf_counter()
        )

    # ----------------------------------------
    # LOOP WATCHDOG
    # ----------------------------------------

    def UpdateLoop(self):

        self.last_loop_time = (
            time.perf_counter()
        )

    # ----------------------------------------
    # FAILSAFE CHECK
    # ----------------------------------------

    def CheckFailsafe(
        self,
        orientation
    ):

        now = time.perf_counter()

        # Controller timeout
        if (
            now - self.last_controller_time
        ) > self.controller_timeout:

            print("FAILSAFE: controller timeout")

            self.failsafe = True

        # Main loop freeze
        if (
            now - self.last_loop_time
        ) > self.loop_timeout:

            print("FAILSAFE: loop timeout")

            self.failsafe = True

        # Excessive tilt
        roll, pitch, yaw = orientation

        if (
            abs(roll) > self.max_angle or
            abs(pitch) > self.max_angle
        ):

            print("FAILSAFE: excessive angle")

            self.failsafe = True

        return self.failsafe

    # ----------------------------------------
    # SHOULD MOTORS RUN?
    # ----------------------------------------

    def MotorsEnabled(self):

        return (
            self.armed and
            not self.failsafe
        )

    # ----------------------------------------
    # RESET FAILSAFE
    # ----------------------------------------

    def ResetFailsafe(self):

        self.failsafe = False