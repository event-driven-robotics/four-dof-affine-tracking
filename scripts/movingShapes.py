import cv2
w = 1920
h = 1080
perc = 0.2
background_filename = "/usr/local/src/affine2dtracking/desk_background.jpg"
shape_filename = "/usr/local/src/affine2dtracking/shapes/military_star.png"
output_filepath = "/data/star_military/translation_y_1920x1080"
motion_type = 2  # 1-> tx, 2-> ty, 3-> rot, 4-> scale, 5-> combined
change_direction = False
sum_tx = 0
sum_ty = 0 
sum_rot = 0
tot_sc = 1
count = 0
count_pass_by_origin = 0

background_image = cv2.imread(background_filename, cv2.IMREAD_COLOR)
shape_image_color = cv2.imread(shape_filename, cv2.IMREAD_COLOR)
shape_image_color = cv2.resize(shape_image_color, (int(perc*w),int(perc*w)))

if (motion_type==3 or motion_type == 4):
    shape_image_color = cv2.copyMakeBorder(shape_image_color, 100, 100, 100, 100, cv2.BORDER_CONSTANT, None, value=0 )

if (motion_type == 5):
    shape_image_color = cv2.copyMakeBorder(shape_image_color, 50, 50, 50, 50, cv2.BORDER_CONSTANT, None, value=0 )

shape_image_grey = cv2.cvtColor(shape_image_color, cv2.COLOR_BGR2GRAY)

resized_background = cv2.resize(background_image, (w,h)); 

initial_position = (int(w/2), int(h/2))

rows_shape_color, cols_shape_color = shape_image_color.shape[:2]

while(1):

    if ( not change_direction):
        if (motion_type==1):
            sum_tx += 1
        if (motion_type==2):
            sum_ty += 1
        if (motion_type==3):
            sum_rot += 0.3 
        if (motion_type==4):
            tot_sc = tot_sc*1.001
    else:
        if (motion_type==1):
            sum_tx -= 1
        if (motion_type==2):
            sum_ty -= 1
        if (motion_type==3): 
            sum_rot -= 0.3
        if (motion_type==4):
            tot_sc = tot_sc*0.999

    new_position = (initial_position[0] + sum_tx, initial_position[1] + sum_ty)

    rot_mat = cv2.getRotationMatrix2D((cols_shape_color/2, rows_shape_color/2), sum_rot, tot_sc)
    warped_shape_image_color = cv2.warpAffine(shape_image_color, rot_mat, (rows_shape_color, cols_shape_color))

    warped_shape_image_grey = cv2.warpAffine(shape_image_grey, rot_mat, (rows_shape_color, cols_shape_color))

    shape_rect, shape_mask = cv2.threshold(warped_shape_image_grey, 40, 255, cv2.THRESH_BINARY_INV); 
    shape_mask_inv = cv2.bitwise_not(shape_mask)        

    rows_mask, cols_mask = shape_mask.shape

    sw = int(cols_shape_color/2)
    sh = int(rows_shape_color/2)

    shape_bg = cv2.bitwise_and(warped_shape_image_color, warped_shape_image_color, mask=shape_mask_inv)
    paste_background_region = resized_background[(new_position[1]-sh):(new_position[1]+sh), (new_position[0]-sw):(new_position[0]+sw)]
    paste_rows, paste_cols = paste_background_region.shape[:2]

    background_fg = cv2.bitwise_and(paste_background_region, paste_background_region, mask=shape_mask[0:paste_rows, 0:paste_cols])

    out_img = cv2.add(shape_bg,background_fg)

    current_background = resized_background.copy()
    current_background[(new_position[1]-sh):(new_position[1]+sh), (new_position[0]-sw):(new_position[0]+sw)] = out_img
    
    if ((sum_tx >= w/2-sw) or (sum_ty >= h/2-sh) or sum_rot > 91 or tot_sc>1.5) and  not change_direction:
        change_direction = True
        print("direction changed")
        sum_sc = 0    
    elif ((sum_tx <= -(w/2-sw) or (sum_ty <= -(h/2-sh)) or sum_rot<-91 or tot_sc<0.5) and change_direction):
        change_direction = False
        print("direction changed")
        sum_sc = 0

    if (count<10):
        cv2.imwrite(output_filepath+"/image-0000"+str(count)+".jpg", current_background)
    elif (count<100):
        cv2.imwrite(output_filepath+"/image-000"+str(count)+".jpg", current_background)
    elif (count<1000):
        cv2.imwrite(output_filepath+"/image-00"+str(count)+".jpg", current_background)
    elif (count<10000):
        cv2.imwrite(output_filepath+"/image-0"+str(count)+".jpg", current_background)
    else:
        cv2.imwrite(output_filepath+"/image-"+str(count)+".jpg", current_background)

    count += 1

    cv2.imshow("final output", current_background)
    cv2.waitKey(1)

    if (motion_type == 1):
        if (new_position[0] == w/2):
            if (count_pass_by_origin==1):
                break 
            count_pass_by_origin+=1
            print("hey") 

    if (motion_type == 2):
        if (new_position[1] == h/2):
            if (count_pass_by_origin==1):
                break
            count_pass_by_origin+=1 
            print("hey") 

    if(motion_type == 3):
        if (sum_rot < 0.01 and sum_rot > -0.01):
            if (count_pass_by_origin==1):
                break
            count_pass_by_origin += 1
            print("hey") 

    if (motion_type == 4):
        if (tot_sc < 1.001 and tot_sc > 1):
            if (count_pass_by_origin==1):
                break
            count_pass_by_origin+=1
            print("hey") 