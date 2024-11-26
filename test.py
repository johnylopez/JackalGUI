import cv2
import numpy as np

width, height = 400, 400
img = np.ones((height, width, 3), dtype=np.uint8) * 255


#Outer Circle
circle_center = (200, 65) 
circle_radius = 60  
circle_color = (174, 175, 177)  
circle_thickness = 3  
cv2.circle(img, circle_center, circle_radius, circle_color, circle_thickness)

#Outer Pipe
pipe_height = 100
pipe_width = 60 
pipe_top_left = (circle_center[0] - pipe_width // 2, circle_center[1] + circle_radius - 10) 
pipe_bottom_right = (circle_center[0] + pipe_width // 2, circle_center[1] + circle_radius + pipe_height)  
pipe_color = (174, 175, 177)  
pipe_thickness = 3 
cv2.rectangle(img, pipe_top_left, pipe_bottom_right, pipe_color, pipe_thickness)

#Inner Pipe
pipe_width2 = 50 
pipe_height2 = 95  
pipe_top_left2 = (circle_center[0] - pipe_width2 // 2, circle_center[1] + circle_radius - 5) 
pipe_bottom_right2 = (circle_center[0] + pipe_width2 // 2, circle_center[1] + circle_radius + pipe_height2)
cv2.rectangle(img, pipe_top_left2, pipe_bottom_right2, (222,223,224), pipe_thickness)  

#Inner Circle
cv2.circle(img, circle_center, 55, (222,223,224), circle_thickness)

#Intersection
small_rect_width = 40  
small_rect_height = 50 
small_rect_top_left = (circle_center[0] - small_rect_width // 2, circle_center[1] + circle_radius - small_rect_height // 2)  #
cv2.rectangle(img, small_rect_top_left, (small_rect_top_left[0] + small_rect_width, small_rect_top_left[1] + small_rect_height), (255, 255, 255), -1)

# Save the image
cv2.imwrite('circle_with_pipe_and_small_white_rect_open_cv.png', img)

# Display the image
cv2.imshow("Circle with Pipe and Small White Rectangle", img)
cv2.waitKey(0)
cv2.destroyAllWindows()