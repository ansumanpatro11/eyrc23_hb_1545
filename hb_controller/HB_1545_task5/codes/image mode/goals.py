import cv2
def read_detected_points(file_path):
    all_x_goals = {}
    all_y_goals = {}
    current_contour_x = []
    current_contour_y = []
    current_contour_idx = None
    
    with open(file_path, "r") as file:
        lines = file.readlines()
        for line in lines:
            if line.startswith("Detected Points for Contour"):
                try:
                    current_contour_idx = int(line.split()[-1][:-1])  # Extract contour index
                    current_contour_x = []  # Reset x-coordinates list
                    current_contour_y = []  # Reset y-coordinates list
                except ValueError:
                    print(f"Error: Unable to parse contour index from line: {line}")
            elif "(" in line and ")" in line:
                # Parse x, y coordinates
                if current_contour_idx is not None:
                    x, y = map(int, line.strip()[1:-1].split(","))
                    current_contour_x.append(x)
                    current_contour_y.append(y)

                    # Update dictionary for the current contour
                    all_x_goals[f"contour{current_contour_idx}"] = current_contour_x
                    all_y_goals[f"contour{current_contour_idx}"] = current_contour_y

    return all_x_goals, all_y_goals
    
    # return current_contour_x,current_contour_y
# File path to the detected points file
detected_points_file = "/home/ansuman/eyrc_HB/eyrc23_hb_1545/hb_controller/HB_1545_task5/codes/saved_contour.txt"

# Read detected points and parse x, y coordinates
x_goals, y_goals = read_detected_points(detected_points_file)
# print((x_goals))
img=cv2.imread('/home/ansuman/eyrc_HB/eyrc23_hb_1545/hb_controller/HB_1545_task5/codes/image mode/elephant.jpg')
# Print the parsed x, y coordinates for each contour
# for contour_name, (x_contour, y_contour) in zip(x_goals.keys(), zip(x_goals.values(), y_goals.values())):
#     print(f"{contour_name}:")
#     for x, y in zip(x_contour, y_contour):
#         # print(f"({x}, {y})")
#         pass
c2_x=[]
c2_y=[]
def get_cont_for_consecutive_contours(contour_num,x_goals,y_goals,n):
    global c2_x,c2_y
    if contour_num==1:
        c2_x=x_goals['contour1'] 
        c2_y=y_goals['contour1']
    else:
        c1_x = x_goals[f'contour{contour_num-1}']
        c1_y = y_goals[f'contour{contour_num-1}']
        
        c2_x = x_goals[f'contour{contour_num}'][len(c1_x):]
        c2_y = y_goals[f'contour{contour_num}'][len(c1_y):]
    
        
    step = int(len(c2_x) / n)
    x_g = []
    y_g = []
    for i in range(0, len(c2_x), step):
        x_g.append(c2_x[i])
        y_g.append(c2_y[i])
    
    return x_g, y_g
# x_g,y_g=get_cont_for_consecutive_contours(detected_points_file,5,26)
# print(f"{x_g}")
# c_2_x,c_2_y=get_c2_for_consecutive_contours(1,x_goals,y_goals,10)

# def sampled_points(c_x,c_y,n):
#     step=int(len(c_x)/n)
#     x_g=[]
#     y_g=[]
#     for i in range(0,len(c_x),step):
#         x_g.append(c_x[i])
#         y_g.append(c_y[i])
    
#     return x_g,y_g


# print(len(c_2_x))

# c_2_x,c_2_y=sampled_points(c_2_x,c_2_y,10)
# print(c_2_x)
# for i in range(len(c_2_x)):
#     x_draw=c_2_x[i]
#     y_draw=c_2_y[i]
#     cv2.circle(img, (x_draw, y_draw), 2, (0, 255, 0), -1)
    
def sampled_points(c_x, c_y, n):
    step = int(len(c_x) / n)
    x_g = []
    y_g = []
    for i in range(0, len(c_x), step):
        x_g.append(c_x[i])
        y_g.append(c_y[i])
    
    return x_g, y_g

# def update_image(*args):
#     global img2, c2_x, c2_y
    
#     n = cv2.getTrackbarPos('Number of Points', 'Image')
#     c= cv2.getTrackbarPos('contour_num', 'Image')
#     c_2_x_sampled, c_2_y_sampled = get_cont_for_consecutive_contours(detected_points_file,c, n)
#     img2 = img.copy() # Reset image
#     for i in range(len(c_2_x_sampled)):
#         x_draw = c_2_x_sampled[i]
#         y_draw = c_2_y_sampled[i]
#         cv2.circle(img2, (x_draw, y_draw),2, (255, 0, 0), -1)
#     cv2.imshow('Image', img2)

# # Assuming you have original_img defined somewhere
# img2 = img.copy()  # Make a copy for drawing

# # Initialize trackbar
# cv2.namedWindow('Image')
# cv2.createTrackbar('Number of Points', 'Image', 1, 100, update_image)
# cv2.createTrackbar('contour_num', 'Image', 1, 25, update_image)


# # Initial call to update_image
# update_image()

# # cv2.waitKey(0)
# # cv2.destroyAllWindows()


# # cv2.imshow('Detected Points', img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()