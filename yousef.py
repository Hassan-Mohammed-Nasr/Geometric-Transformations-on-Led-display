import time
import math
import numpy as np
import RPi.GPIO as GPIO
from rgbmatrix import RGBMatrix, RGBMatrixOptions

# ===========================================================
# MATRIX SETUP (32x32)
# ===========================================================

options = RGBMatrixOptions()
options.hardware_mapping = 'adafruit-hat'
options.rows = 32
options.cols = 32
options.chain_length = 1

matrix = RGBMatrix(options=options)

WIDTH = 32
HEIGHT = 32

# ===========================================================
# BUTTON SETUP
# ===========================================================

BUTTON_PIN = 25

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# ===========================================================
# SQUARE DEFINITION (16x16, centered at origin)
# ===========================================================

SQUARE_SIZE = 16
half = SQUARE_SIZE // 2

xs = np.arange(-half, half)
ys = np.arange(-half, half)
X, Y = np.meshgrid(xs, ys)
X = X.ravel()
Y = Y.ravel()

ones = np.ones_like(X)
P_local = np.vstack([X, Y, ones])   # 3xN

CX = WIDTH / 2.0
CY = HEIGHT / 2.0

# ===========================================================
# TRANSFORMATION MATRICES
# ===========================================================

def T_translate(bx, by):
    return np.array([
        [1.0, 0.0, bx],
        [0.0, 1.0, by],
        [0.0, 0.0, 1.0]
    ])

def T_rotate(theta):
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array([
        [ c, -s, 0.0],
        [ s,  c, 0.0],
        [0.0, 0.0, 1.0]
    ])

def T_scale(dx, dy):
    return np.array([
        [dx, 0.0, 0.0],
        [0.0, dy, 0.0],
        [0.0, 0.0, 1.0]
    ])

def T_shear(dx, dy):
    return np.array([
        [1.0, dx, 0.0],
        [dy, 1.0, 0.0],
        [0.0, 0.0, 1.0]
    ])

T_to_center = T_translate(CX, CY)

# ===========================================================
# DRAWING FUNCTION
# ===========================================================

def draw_points(P, color):
    matrix.Clear()
    xs = np.rint(P[0]).astype(int)
    ys = np.rint(P[1]).astype(int)
    r, g, b = color

    for x, y in zip(xs, ys):
        if 0 <= x < WIDTH and 0 <= y < HEIGHT:
            matrix.SetPixel(x, y, r, g, b)

# ===========================================================
# TRANSFORMATION DEMOS (ONE STEP EACH)
# ===========================================================

def demo_translation():
    for bx in np.linspace(-6, 6, 25):
        T = T_to_center @ T_translate(bx, 0)
        draw_points(T @ P_local, (255, 0, 0))
        time.sleep(0.05)

def demo_rotation():
    for angle in range(0, 360, 5):
        theta = math.radians(angle)
        T = T_to_center @ T_rotate(theta)
        draw_points(T @ P_local, (0, 255, 0))
        time.sleep(0.03)

def demo_scaling():
    for s in np.linspace(0.5, 1.5, 25):
        T = T_to_center @ T_scale(s, s)
        draw_points(T @ P_local, (0, 0, 255))
        time.sleep(0.05)

def demo_shearing():
    for dx in np.linspace(0.0, 1.0, 25):
        T = T_to_center @ T_shear(dx, 0)
        draw_points(T @ P_local, (255, 255, 0))
        time.sleep(0.05)

# ===========================================================
# MODE CONTROL
# ===========================================================

MODES = [
    demo_translation,
    demo_rotation,
    demo_scaling,
    demo_shearing
]

MODE_NAMES = [
    "TRANSLATION",
    "ROTATION",
    "SCALING",
    "SHEARING"
]

# ===========================================================
# MAIN LOOP
# ===========================================================

def main():
    current_mode = 0
    last_button = GPIO.input(BUTTON_PIN)

    try:
        while True:
            button = GPIO.input(BUTTON_PIN)

            # Button press detection
            if last_button == GPIO.HIGH and button == GPIO.LOW:
                current_mode = (current_mode + 1) % len(MODES)
                matrix.Clear()
                time.sleep(0.3)  # debounce

            last_button = button

            # Run current mode
            MODES[current_mode]()

    except KeyboardInterrupt:
        pass
    finally:
        matrix.Clear()
        GPIO.cleanup()

# ===========================================================
# ENTRY POINT
# ===========================================================

if __name__ == "__main__":
    main()