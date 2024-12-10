import cv2
import numpy as np

# Deficiency log with position and deficiency text
deficiencies_log = {
    "35.2": {"detections": "fracture", "image": np.ones((20, 20, 3), dtype=np.uint8) * 180},  # Example image
    "36.7": {"detections": "crack", "image": np.ones((20, 20, 3), dtype=np.uint8) * 200},
    "38.5": {"detections": "leak", "image": np.ones((20, 20, 3), dtype=np.uint8) * 100},
    "38.7": {"detections": "leak", "image": np.ones((20, 20, 3), dtype=np.uint8) * 100},
    "38.9": {"detections": "leak", "image": np.ones((20, 20, 3), dtype=np.uint8) * 100},
    "50.5": {"detections": "leak", "image": np.ones((20, 20, 3), dtype=np.uint8) * 100},

}

numberDeficiencies = len(deficiencies_log)
width, height = 1000, (350 + 300*numberDeficiencies)
img = np.ones((height, width, 3), dtype=np.uint8) * 255

center = int(width / 2)
circle_center = (center + 250, 80)
circle_radius = 60
circle_color = (174, 175, 177)
circle_thickness = 3
cv2.circle(img, circle_center, circle_radius, circle_color, circle_thickness)

pipe_height = 300 * numberDeficiencies
pipe_width = 60
pipe_top_left = (circle_center[0] - pipe_width // 2, circle_center[1] + circle_radius - 10)
pipe_bottom_right = (circle_center[0] + pipe_width // 2, circle_center[1] + circle_radius + pipe_height)
pipe_color = (174, 175, 177)
pipe_thickness = 3
cv2.rectangle(img, pipe_top_left, pipe_bottom_right, pipe_color, pipe_thickness)

pipe_width2 = 50
pipe_height2 = (300 * numberDeficiencies) - 5
pipe_top_left2 = (circle_center[0] - pipe_width2 // 2, circle_center[1] + circle_radius - 5)
pipe_bottom_right2 = (circle_center[0] + pipe_width2 // 2, circle_center[1] + circle_radius + pipe_height2)
cv2.rectangle(img, pipe_top_left2, pipe_bottom_right2, (222,223,224), pipe_thickness)

cv2.circle(img, circle_center, 55, (222,223,224), circle_thickness)

small_rect_width = 40
small_rect_height = 50
small_rect_top_left = (circle_center[0] - small_rect_width // 2, circle_center[1] + circle_radius - small_rect_height // 2)
cv2.rectangle(img, small_rect_top_left, (small_rect_top_left[0] + small_rect_width, small_rect_top_left[1] + small_rect_height), (255, 255, 255), -1)

vertical_pos = circle_center[1] + 200
y_offset = 300
new_image_size = 250  # Large image size

# Adjust the width for placement of large images (250x250)
horizontal_offset = new_image_size // 2  # Set this to half of the new image size for correct alignment

for position_str, value in deficiencies_log.items():
    position = float(position_str)
    detection = value["detections"]
    img_to_overlay = value["image"]
    pos_ft = position_str + "ft"

    # Add text for position and detection
    cv2.putText(img, pos_ft, (center - 400, vertical_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(img, detection, (center - 300, vertical_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

    # Resize the deficiency image to the desired size
    img_thumbnail = cv2.resize(img_to_overlay, (new_image_size, new_image_size))

    # Calculate the correct position for the large image
    # Ensure image fits inside the canvas width
    img[y_offset - new_image_size // 2:y_offset + new_image_size // 2,
        center - horizontal_offset:center + horizontal_offset] = img_thumbnail

    vertical_pos += 300
    y_offset += 300


cv2.imwrite('circle_with_pipe_and_deficiencies_resized.png', img)

# for position_str, value in deficiencies_log.items():
#     position = float(position_str)
#     detection = value["detections"]
#     img_to_overlay = value["image"]
#     pos_ft = position_str + "ft"

#     cv2.putText(img, pos_ft, (circle_center[0] - 400, vertical_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
#     cv2.putText(img, detection, (circle_center[0] - 300, vertical_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

#     img_thumbnail = cv2.resize(img_to_overlay, (50, 50))
#     img[y_offset - 25:y_offset + 25, circle_center[0] - 100:circle_center[0] - 50] = img_thumbnail
#     vertical_pos += 100
#     y_offset += 100

# cv2.imwrite('circle_with_pipe_and_deficiencies4.png', img)

