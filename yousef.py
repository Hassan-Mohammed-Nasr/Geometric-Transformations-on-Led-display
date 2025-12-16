import time
import math
import numpy as np
import RPi.GPIO as GPIO
from rgbmatrix import RGBMatrix, RGBMatrixOptions

# ===========================================================
# MATRIX SETUP
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

BUTTON_PIN = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# ===========================================================
# GLOBAL STATE
# ===========================================================

mode_changed = False
current_mode = 0

# ===========================================================
# BUTTON CALLBACK
# ===========================================================

def button_pressed(channel):
    global current_mode, mode_changed
    current_mode = (current_mode + 1) % 4
    mode_changed = True

GPIO.add_event_detect(
    BUTTON_PIN,
    GPIO.FALLING,
    callback=button_pressed,
    bouncetime=300
)

# ===========================================================
# GEOMETRY SETUP
# ===========================================================

SQUARE_SIZE = 16
half = SQUARE_SIZE // 2

xs = np.arange(-half, half)
ys = np.arange(-half, half)
X, Y = np.meshgrid(xs, ys)
P_local = np.vstack([X.ravel(), Y.ravel(), np.ones(X.size)])

CX = WIDTH / 2.0
CY = HEIGHT / 2.0

# ===========================================================
# TRANSFORMS
# ===========================================================

def T_translate(bx, by):
    return np.array([[1,0,bx],[0,1,by],[0,0,1]])

def T_rotate(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])

def T_scale(sx, sy):
    return np.array([[sx,0,0],[0,sy,0],[0,0,1]])

def T_shear(dx, dy):
    return np.array([[1,dx,0],[dy,1,0],[0,0,1]])

T_to_center = T_translate(CX, CY)

# ===========================================================
# DRAW
# ===========================================================

def draw(P, color):
    matrix.Clear()
    xs = np.rint(P[0]).astype(int)
    ys = np.rint(P[1]).astype(int)
    for x, y in zip(xs, ys):
        if 0 <= x < WIDTH and 0 <= y < HEIGHT:
            matrix.SetPixel(x, y, *color)

# ===========================================================
# ANIMATIONS (INTERRUPTIBLE)
# ===========================================================

def anim_translation():
    for bx in np.linspace(-6, 6, 30):
        if mode_changed: return
        draw(T_to_center @ T_translate(bx,0) @ P_local, (255,0,0))
        time.sleep(0.05)

def anim_rotation():
    for a in range(0,360,5):
        if mode_changed: return
        draw(T_to_center @ T_rotate(math.radians(a)) @ P_local, (0,255,0))
        time.sleep(0.03)

def anim_scaling():
    for s in np.linspace(0.5,1.5,30):
        if mode_changed: return
        draw(T_to_center @ T_scale(s,s) @ P_local, (0,0,255))
        time.sleep(0.05)

def anim_shearing():
    for dx in np.linspace(0,1,30):
        if mode_changed: return
        draw(T_to_center @ T_shear(dx,0) @ P_local, (255,255,0))
        time.sleep(0.05)

ANIMS = [
    anim_translation,
    anim_rotation,
    anim_scaling,
    anim_shearing
]

# ===========================================================
# MAIN
# ===========================================================

def main():
    global mode_changed

    try:
        while True:
            mode_changed = False
            ANIMS[current_mode]()
            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        matrix.Clear()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
