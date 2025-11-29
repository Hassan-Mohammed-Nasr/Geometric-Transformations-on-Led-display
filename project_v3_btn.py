from rgbmatrix import RGBMatrix, RGBMatrixOptions
from RPi import GPIO
import time
import numpy as np


## Configuration
TASK = 'deliverable_1'
TRANSFORMATION = 'scaling'

ROWS = 32
COLS = 32 

options = RGBMatrixOptions()
options.rows = ROWS
options.cols = COLS
options.hardware_mapping = 'adafruit-hat'

GPIO.setmode(GPIO.BCM)
button_pin = 25  
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)




def change_transformation(trans):
    transformations = ['translation', 'rotation', 'scaling', 'shearing']
    current = transformations.index(trans)
    return transformations[(current + 1) % len(transformations)]


def Translation(dx, dy):
    return np.array([[1, 0, dx],
                     [0, 1, dy],
                     [0, 0, 1]], dtype=float)

def Rotation(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]], dtype=float)

def Scaling(sx, sy):
    return np.array([[sx, 0,  0],
                     [0,  sy, 0],
                     [0,  0,  1]], dtype=float)

def Shearing(shx, shy):
    return np.array([[1,   shx, 0],
                     [shy, 1,   0],
                     [0,   0,   1]], dtype=float)



## TASK 1: Light All LEDs
if TASK == 'deliverable_1':
    '''
    Light all LEDs on the matrix with white color
    Abdulrahman Al Ward.
    '''
    print("Executing Deliverable 1: Light All LEDs")
    matrix = RGBMatrix(options = options)
    for x in range(ROWS):
        for y in range(COLS):
            matrix.SetPixel(x,y, 255, 255, 255)


## TASK 2: Light Single Pixel
elif TASK == 'deliverable_2':
    '''
    Light a single pixel at the center of the matrix
    Hassan Mohammed Nasr.
    '''
    print("Executing Deliverable 2: Light Single Pixel")
    matrix = RGBMatrix(options = options)
    matrix.SetPixel(16,16, 255, 255, 255)  


## Task 3: Different intensity rows
elif TASK == 'deliverable_3':
    '''
    Light each row with increasing intensity from left to right
    Abdulrahman Al Ward.
    '''
    
    print("Executing Deliverable 3: Different intensity rows")
    matrix = RGBMatrix(options = options)
    for x in range(ROWS):  
        for y in range(COLS):
            intensity = int((y / (COLS - 1)) * 255)
            matrix.SetPixel(x, y, intensity, intensity, intensity)


## TASK 4: Light Square
elif TASK == 'deliverable_4':
    '''
    Light a 12x12 square in the center of the matrix
    Hassan Mohammed Nasr.
    '''

    print("Executing Deliverable 4: Light Square")
    matrix = RGBMatrix(options = options)
    for x in range(10, 22):
        for y in range(10, 22):
            matrix.SetPixel(x,y, 255, 255, 255)




## TASK: DEMO (Geometric Transformations) #### Under Modifications

elif TASK == 'demo':
    print("Executing Demo Task")

    MxN = 12*12 
    SECONDS_DELAY = 0.1 
    ITERATIONS = 100
    CURRENT_TRANSFORMATION = TRANSFORMATION
    matrix = RGBMatrix(options = options)
    prev_button_state = GPIO.LOW

    while True:
        
        square_matrix = np.zeros((3, MxN), dtype=float)
        
        # Create a square in the center and extract the points
        i = 0
        for x in range(10, 22):
            for y in range(10, 22):
                matrix.SetPixel(x,y, 255, 255, 255)
                square_matrix[0,i] = x
                square_matrix[1,i] = y
                square_matrix[2,i] = 1
                i += 1

        

        if CURRENT_TRANSFORMATION == 'translation':

            cx, cy  = 1, 1

            for _ in range (ITERATIONS):
                current_button_state = GPIO.input(button_pin)

                if current_button_state == GPIO.HIGH and prev_button_state == GPIO.LOW:
                    CURRENT_TRANSFORMATION = change_transformation(CURRENT_TRANSFORMATION)
                    print(f"Changed transformation to: {CURRENT_TRANSFORMATION}")
                    prev_button_state = current_button_state 
                    time.sleep(0.3)  
                    break
                prev_button_state = current_button_state  

                top_x_left = int(square_matrix[0,:].min())
                top_y_left = int(square_matrix[1,:].min())
                bottom_x_right = int(square_matrix[0,:].max())
                bottom_y_right = int(square_matrix[1,:].max())

                hit_x = top_x_left + cx < 0 or bottom_x_right + cx >= ROWS
                hit_y = top_y_left + cy < 0 or bottom_y_right + cy >= COLS
                if hit_x: 
                    cx = -cx
                if hit_y: 
                    cy = -cy

                translation_matrix = Translation(cx, cy)
                square_matrix = translation_matrix @ square_matrix

                matrix.Clear()
                for i in range(MxN):
                    x, y = int(square_matrix[0,i]), int(square_matrix[1,i])
                    matrix.SetPixel(x, y, 255, 255, 255)
                time.sleep(SECONDS_DELAY)

        elif CURRENT_TRANSFORMATION == 'rotation':

            rotation_angle = np.deg2rad(3)
            center_x, center_y =  square_matrix[0,:].mean(), square_matrix[1,:].mean()
            
            # Helper function to fill polygon between 4 points
            def fill_polygon(matrix, corners):
                """Fill a quadrilateral defined by 4 corner points"""
                # Extract corner coordinates
                x_coords = [int(np.rint(corners[0, i])) for i in range(4)]
                y_coords = [int(np.rint(corners[1, i])) for i in range(4)]
                
                # Find bounding box
                min_x = max(0, min(x_coords))
                max_x = min(ROWS - 1, max(x_coords))
                min_y = max(0, min(y_coords))
                max_y = min(COLS - 1, max(y_coords))
                
                # Use scanline fill algorithm
                for y in range(min_y, max_y + 1):
                    intersections = []
                    for i in range(4):
                        j = (i + 1) % 4
                        x1, y1 = x_coords[i], y_coords[i]
                        x2, y2 = x_coords[j], y_coords[j]
                        
                        if y1 <= y < y2 or y2 <= y < y1:
                            if y2 != y1:
                                x_intersect = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
                                intersections.append(x_intersect)
                    
                    intersections.sort()
                    for i in range(0, len(intersections), 2):
                        if i + 1 < len(intersections):
                            x_start = max(0, int(np.rint(intersections[i])))
                            x_end = min(ROWS - 1, int(np.rint(intersections[i + 1])))
                            for x in range(x_start, x_end + 1):
                                matrix.SetPixel(x, y, 255, 255, 255)
            
            # Extract only the 4 corners from the square
            corners_indices = [0, 11, 143, 132]  # top-left, top-right, bottom-right, bottom-left
            corner_matrix = square_matrix[:, corners_indices]

            for _ in range (ITERATIONS):
                current_button_state = GPIO.input(button_pin)

                if current_button_state == GPIO.HIGH and prev_button_state == GPIO.LOW:
                    CURRENT_TRANSFORMATION = change_transformation(CURRENT_TRANSFORMATION)
                    print(f"Changed transformation to: {CURRENT_TRANSFORMATION}")
                    prev_button_state = current_button_state 
                    time.sleep(0.3)  
                    break
                prev_button_state = current_button_state 
        
                transformation = Translation(center_x, center_y) @ Rotation(rotation_angle) @ Translation(-center_x, -center_y)
                corner_matrix = transformation @ corner_matrix
                
                # Update center based on transformed corners
                center_x, center_y = corner_matrix[0,:].mean(), corner_matrix[1,:].mean()
                
                matrix.Clear()
                fill_polygon(matrix, corner_matrix)
                time.sleep(SECONDS_DELAY)
                

        elif CURRENT_TRANSFORMATION == 'scaling':

            cx, cy  = 1.02, 1.02


            for _ in range (ITERATIONS):
                current_button_state = GPIO.input(button_pin)

                if current_button_state == GPIO.HIGH and prev_button_state == GPIO.LOW:
                    CURRENT_TRANSFORMATION = change_transformation(CURRENT_TRANSFORMATION)
                    print(f"Changed transformation to: {CURRENT_TRANSFORMATION}")
                    prev_button_state = current_button_state  
                    time.sleep(0.3)  
                    break
                prev_button_state = current_button_state

                center_x, center_y =  square_matrix[0,:].mean(), square_matrix[1,:].mean()

                predict = Translation(center_x, center_y) @ Scaling(cx, cy) @ Translation(-center_x, -center_y) @ square_matrix
                
                top_x_left = int(predict[0,:].min())
                top_y_left = int(predict[1,:].min())
                bottom_x_right = int(predict[0,:].max())
                bottom_y_right = int(predict[1,:].max())

                hit_x = top_x_left  < 0 or bottom_x_right  >= ROWS
                hit_y = top_y_left  < 0 or bottom_y_right  >= COLS

                if hit_x: 
                    cx = 1/cx
                    predict = Translation(center_x, center_y) @ Scaling(cx, cy) @ Translation(-center_x, -center_y) @ square_matrix
                if hit_y: 
                    cy = 1/cy
                    predict = Translation(center_x, center_y) @ Scaling(cx, cy) @ Translation(-center_x, -center_y) @ square_matrix


                square_matrix = predict

                top_x_left = int(square_matrix[0,:].min())
                top_y_left = int(square_matrix[1,:].min())
                bottom_x_right = int(square_matrix[0,:].max())
                bottom_y_right = int(square_matrix[1,:].max())


                matrix.Clear()
                square_points = []
                for i in range(MxN):
                    x, y = int(np.rint(square_matrix[0,i])), int(np.rint(square_matrix[1,i]))
                    if 0 <= x < ROWS and 0 <= y < COLS:
                        square_points.append( (x,y) )
                        matrix.SetPixel(x, y, 255, 255, 255)


                for x in range(max(0, top_x_left), min(ROWS, bottom_x_right + 1)):
                    for y in range(max(0, top_y_left), min(COLS, bottom_y_right + 1)):
                        if (x, y) not in square_points:
                            matrix.SetPixel(x, y, 255, 255, 255)
                            
                    
                    
                time.sleep(SECONDS_DELAY)
                


        elif CURRENT_TRANSFORMATION == 'shearing':

            cx, cy  = 0, 0.02
            
            for _ in range (ITERATIONS):
                current_button_state = GPIO.input(button_pin)

                if current_button_state == GPIO.HIGH and prev_button_state == GPIO.LOW:
                    CURRENT_TRANSFORMATION = change_transformation(CURRENT_TRANSFORMATION)
                    print(f"Changed transformation to: {CURRENT_TRANSFORMATION}")
                    prev_button_state = current_button_state  
                    time.sleep(0.3) 
                    break
                prev_button_state = current_button_state 

                center_x, center_y =  square_matrix[0,:].mean(), square_matrix[1,:].mean()
                

                predict = Translation(center_x, center_y) @ Shearing(cx, cy) @ Translation(-center_x, -center_y) @ square_matrix
                top_x_left = int(predict[0,:].min())
                top_y_left = int(predict[1,:].min())
                bottom_x_right = int(predict[0,:].max())
                bottom_y_right = int(predict[1,:].max())

                hit_x = top_x_left  < 0 or bottom_x_right  >= ROWS
                hit_y = top_y_left  < 0 or bottom_y_right  >= COLS

                if hit_x: 
                    cx = -cx
                    predict = Translation(center_x, center_y) @ Shearing(cx, cy) @ Translation(-center_x, -center_y) @ square_matrix
                if hit_y: 
                    cy = -cy
                    predict = Translation(center_x, center_y) @ Shearing(cx, cy) @ Translation(-center_x, -center_y) @ square_matrix
                
                square_matrix = predict



                matrix.Clear()
                for i in range(MxN):
                    x, y = int(np.rint(square_matrix[0,i])), int(np.rint(square_matrix[1,i]))
                    if 0 <= x < ROWS and 0 <= y < COLS:
                        matrix.SetPixel(x, y, 255, 255, 255)
                time.sleep(SECONDS_DELAY)


## Clear the matrix after a delay
time.sleep(10)
matrix.Clear()