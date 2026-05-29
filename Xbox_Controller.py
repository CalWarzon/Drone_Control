#import evdev
#from evdev import InputDevice, ecodes
import select


class XboxController:
    def __init__(self, name="Xbox"):
        #self.device = self.FindDevice(name)

        # State (normalized)
        self.state = {
            # Sticks
            "lx": 0.0, "ly": 0.0,
            "rx": 0.0, "ry": 0.0,

            # Triggers
            "lt": 0.0, "rt": 0.0,

            # Buttons
            "a": 0, "b": 0, "x": 0, "y": 0,
            "lb": 0, "rb": 0,
            "back": 0, "start": 0,
            "guide": 0,

            # D-pad
            "dpad_x": 0,
            "dpad_y": 0,

            # Stick press
            "ls": 0, "rs": 0
        }

    # ---------------------------
    # DEVICE DETECTION
    # ---------------------------
    def FindDevice(self, name):
        devices = [InputDevice(path) for path in evdev.list_devices()]

        for dev in devices:
            if name.lower() in dev.name.lower():
                print(f"Connected to: {dev.path} ({dev.name})")
                dev.grab()  # exclusive access (optional)
                return dev

        raise RuntimeError("Xbox controller not found")

    # ---------------------------
    # NORMALIZATION
    # ---------------------------
    def NormalizeStick(self, value):
        if value >= 0:
            return value / 32767.0 - 1
        return value / 32768.0 - 1

    def NormalizeTrigger(self, value):
        return value / 1023.0

    def Deadzone(self, value, dz=0.12):
        if abs(value) < dz:
            return 0.0
        return (value - dz * (1 if value > 0 else -1)) / (1 - dz)

    def Smooth(self, old, new, alpha=0.25):
        return old + alpha * (new - old)

    # ---------------------------
    # READ INPUT (NON-BLOCKING)
    # ---------------------------
    def Read(self):
        try:
            r, _, _ = select.select([self.device.fd], [], [], 0)

            if r:
                while True:
                    event = self.device.read_one()

                    if event is None:
                        break

                    if event.type == ecodes.EV_ABS:
                        self.HandleAbs(event)

                    elif event.type == ecodes.EV_KEY:
                        self.HandleKey(event)

        except OSError:
            print("Controller disconnected!")
            return None

        return self.state

    # ---------------------------
    # HANDLE ANALOG INPUTS
    # ---------------------------
    def HandleAbs(self, event):
        code = event.code
        val = event.value

        if code == ecodes.ABS_X:
            self.state["lx"] = self.Deadzone(self.NormalizeStick(val))

        elif code == ecodes.ABS_Y:
            self.state["ly"] = self.Deadzone(-self.NormalizeStick(val))

        elif code == ecodes.ABS_Z:
            self.state["rx"] = self.Deadzone(self.NormalizeStick(val))

        elif code == ecodes.ABS_RZ:
            self.state["ry"] = self.Deadzone(-self.NormalizeStick(val))

        elif code == ecodes.ABS_BRAKE:   # LT
            self.state["lt"] = self.NormalizeTrigger(val)

        elif code == ecodes.ABS_GAS:  # RT
            self.state["rt"] = self.NormalizeTrigger(val)

        elif code == ecodes.ABS_HAT0X:
            self.state["dpad_x"] = val  # -1 left, 1 right

        elif code == ecodes.ABS_HAT0Y:
            self.state["dpad_y"] = val  # -1 up, 1 down

    # ---------------------------
    # HANDLE BUTTON INPUTS
    # ---------------------------
    def HandleKey(self, event):
        val = event.value
        code = event.code

        mapping = {
            ecodes.BTN_SOUTH: "a",
            ecodes.BTN_EAST: "b",
            ecodes.BTN_WEST: "y",
            ecodes.BTN_NORTH: "x",

            ecodes.BTN_TL: "lb",
            ecodes.BTN_TR: "rb",

            ecodes.BTN_SELECT: "back",
            ecodes.BTN_START: "start",
            ecodes.BTN_MODE: "guide",

            ecodes.BTN_THUMBL: "ls",
            ecodes.BTN_THUMBR: "rs"
        }

        if code in mapping:
            self.state[mapping[code]] = val