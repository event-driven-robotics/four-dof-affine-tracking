import cv2
import numpy as np
import math
from dataclasses import dataclass

w = 1920
h = 1080
perc = 0.3
background_filename = "/home/lgava-iit.local/code/four-dof-affine-tracking/backgrounds/desk_background.jpg"
shape_filename = "/home/lgava-iit.local/code/four-dof-affine-tracking/shapes/mustard-removebg.png"
output_filepath = "/home/lgava-iit.local/data/real-objects-open-loop/mustard/scale_1920x1080"
motion_type = 4  # 1-> tx, 2-> ty, 3-> rot, 4-> scale, 5-> combined
background_dynamic = True

sum_tx = 0
sum_ty = 0
sum_rot = 0
tot_sc = 1
count = 0
count_pass_by_origin = 0
change_direction = True

@dataclass
class AffineState:
    x: float
    y: float
    d: float
    s: float
    k: float

    def __add__(self, rhs):
        return AffineState(
            self.x + rhs.x,
            self.y + rhs.y,
            self.d + rhs.d,
            self.s + rhs.s,
            self.k
        )

    def __sub__(self, rhs):
        return AffineState(
            self.x - rhs.x,
            self.y - rhs.y,
            self.d - rhs.d,
            self.s - rhs.s,
            self.k
        )

    def __mul__(self, t: float):
        return AffineState(
            self.x * t,
            self.y * t,
            self.d * t,
            self.s * t,
            self.k
        )

    __rmul__ = __mul__   # allows: j * state

waypoints = []

waypoints = [
    AffineState(0.0, 0.0, 0.0, 1.0, 0.0),
    AffineState(100.0, 200.0, 45.0, 1.3, 0.0),
    AffineState(320.0, 30.0, -22.0, 0.9, 0.0),
    AffineState(600.0, 100.0, -22.0, 0.8, 0.0),
    AffineState(100.0, -200.0, -10.0, 1.0, 0.0),
    AffineState(-200.0, -250.0, 20.0, 1.2, 0.0),
    AffineState(-400.0, 50.0, 10.0, 0.95, 0.0),
    AffineState(-550.0, 200.0, 5.0, 0.9, 0.0),
    AffineState(-300.0, 150.0, 2.0, 1.1, 0.0),
    AffineState(0.0, 0.0, 0.0, 1.0, 0.0),
]

def interpolate_states(waypoints, n: int):
    k_in = len(waypoints)
    k_out = (k_in - 1) * n

    output = []

    for i in range(k_out):
        i_in = i // n
        j = (i % n) / float(n)
        interp = waypoints[i_in] + (waypoints[i_in + 1] - waypoints[i_in]) * j
        output.append(interp)

    output.append(waypoints[-1])
    return output

interpolated = interpolate_states(waypoints, 250)

# Load images
background_image = cv2.imread(background_filename, cv2.IMREAD_COLOR)
shape_image_color = cv2.imread(shape_filename, cv2.IMREAD_COLOR)
shape_image_color = cv2.resize(shape_image_color, (int(perc * w), int(perc * w)))

if motion_type == 3 or motion_type == 4:
    shape_image_color = cv2.copyMakeBorder(shape_image_color, 50, 50, 50, 50, cv2.BORDER_CONSTANT, value=0)

if motion_type == 5:
    shape_image_color = cv2.copyMakeBorder(shape_image_color, 50, 50, 50, 50, cv2.BORDER_CONSTANT, value=0)

shape_image_grey = cv2.cvtColor(shape_image_color, cv2.COLOR_BGR2GRAY)

resized_background = cv2.resize(background_image, (w, h))
moving_background = cv2.resize(background_image, (w + 200, h + 200))  # larger for moving region

initial_position = (int(w / 2), int(h / 2))
rows_shape_color, cols_shape_color = shape_image_color.shape[:2]

angle_list = np.arange(0, 360, 0.1)
radius = 100
random_x = [int(radius * math.cos(a * math.pi / 180)) for a in angle_list]
random_y = [int(radius * math.sin(a * math.pi / 180)) for a in angle_list]
background_counter = 0

while True:
    # update transformation parameters

    if motion_type == 5:
        if count >= len(interpolated):
            break
        sum_tx = interpolated[count].x
        sum_ty = interpolated[count].y
        sum_rot = interpolated[count].d
        tot_sc  = interpolated[count].s
    else:
        if not change_direction:
            if motion_type == 1:
                sum_tx += 1
            elif motion_type == 2:
                sum_ty += 1
            elif motion_type == 3:
                sum_rot += 0.3
            elif motion_type == 4:
                tot_sc *= 1.001
        else:
            if motion_type == 1:
                sum_tx -= 1
            elif motion_type == 2:
                sum_ty -= 1
            elif motion_type == 3:
                sum_rot -= 0.3
            elif motion_type == 4:
                tot_sc *= 0.999

    cx = int(initial_position[0] + sum_tx)
    cy = int(initial_position[1] + sum_ty)
    new_position = (cx, cy)

    # apply transformation to shape
    rot_mat = cv2.getRotationMatrix2D((cols_shape_color / 2, rows_shape_color / 2), sum_rot, tot_sc)
    warped_shape_image_color = cv2.warpAffine(shape_image_color, rot_mat, (cols_shape_color, rows_shape_color))
    warped_shape_image_grey = cv2.warpAffine(shape_image_grey, rot_mat, (cols_shape_color, rows_shape_color))

    shape_mask = cv2.threshold(warped_shape_image_grey, 40, 255, cv2.THRESH_BINARY_INV)[1]
    shape_mask_inv = cv2.bitwise_not(shape_mask)

    sw = int(cols_shape_color / 2)
    sh = int(rows_shape_color / 2)

    shape_bg = cv2.bitwise_and(warped_shape_image_color, warped_shape_image_color, mask=shape_mask_inv)

    # compute moving background
    if background_dynamic:
        dx = random_x[background_counter % len(random_x)]
        dy = random_y[background_counter % len(random_y)]
        background_counter += 1

        # crop region from moving background
        x_offset = 100 + dx
        y_offset = 100 + dy
        x1 = max(0, x_offset)
        x2 = min(moving_background.shape[1], x_offset + w)
        y1 = max(0, y_offset)
        y2 = min(moving_background.shape[0], y_offset + h)

        current_background = moving_background[y1:y2, x1:x2].copy()
    else:
        current_background = resized_background.copy()

    # print(new_position[0], new_position[1], sw, sh)

    x_start = max(0, cx - sw)
    x_end   = min(current_background.shape[1], cx + sw)
    y_start = max(0, cy - sh)
    y_end   = min(current_background.shape[0], cy + sh)

    shape_x_start = max(0, sw - cx)             # shift if the shape is partially offscreen
    shape_x_end   = shape_x_start + (x_end - x_start)
    shape_y_start = max(0, sh - cy)
    shape_y_end   = shape_y_start + (y_end - y_start)

    paste_bg_region = current_background[y_start:y_end, x_start:x_end]

    bg_fg = cv2.bitwise_and(paste_bg_region, paste_bg_region,
                            mask=shape_mask[shape_y_start:shape_y_end,
                                            shape_x_start:shape_x_end])

    out_img = cv2.add(shape_bg[shape_y_start:shape_y_end,
                            shape_x_start:shape_x_end], bg_fg)

    current_background[y_start:y_end, x_start:x_end] = out_img

    # Check bounds for direction change
    if ((sum_tx >= w / 2 - sw or sum_ty >= h / 2 - sh or sum_rot > 91 or tot_sc > 1.5) and not change_direction):
        change_direction = True
        print("direction changed true")
    elif ((sum_tx <= -(w / 2 - sw) or sum_ty <= -(h / 2 - sh) or sum_rot < -91 or tot_sc < 0.5) and change_direction):
        change_direction = False
        print("direction changed false")

    # Save frame
    filename = output_filepath + f"/image-{count:05d}.jpg"
    # cv2.imshow("image",current_background)
    # cv2.waitKey(1)
    cv2.imwrite(filename, current_background)

    # cv2.imshow("final output", current_background)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break

    # stop when the shape passes origin again
    if motion_type == 1 and new_position[0] == w / 2:
        if count_pass_by_origin == 1:
            break
        count_pass_by_origin += 1

    if motion_type == 2 and new_position[1] == h / 2:
        if count_pass_by_origin == 1:
            break
        count_pass_by_origin += 1

    if motion_type == 3 and -0.01 < sum_rot < 0.01:
        if count_pass_by_origin == 1:
            break
        count_pass_by_origin += 1

    if motion_type == 4 and 1 < tot_sc < 1.001:
        if count_pass_by_origin == 1:
            break
        count_pass_by_origin += 1

    count += 1

            

# cv2.destroyAllWindows()
