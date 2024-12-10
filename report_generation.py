import numpy as np
from collections import defaultdict
import cv2

# Function to group positions into 5ft intervals
def group_by_5ft(position):
    position = float(position)
    return int(position // 5) * 5  # Returns the lower bound of the interval

def create_log(deficiencies_log):
    # Initialize the dictionary to store deficiency counts by 5ft intervals
    deficiency_counts = defaultdict(lambda: defaultdict(int))
    # Dictionary to store the image tensors for each interval
    interval_images = defaultdict(list)

    # Group the deficiencies by 5ft interval and count their occurrences
    for position_str, value in deficiencies_log.items():
        position = float(position_str)  # Convert the key (position) back to a float
        detections = value["detections"]  # Get the detections for this position
        image = value["image"]  # Get the image tensor for this position
        
        # If detections is a string, wrap it in a list to make it iterable
        if isinstance(detections, str):
            detections = [detections]

        interval = group_by_5ft(position)  # Get the 5ft interval
        
        # Count deficiencies in this interval
        for detection in detections:
            deficiency_counts[interval][detection] += 1

        # Save the image tensor associated with this interval
        interval_images[interval].append(image)

    # Find the most recurrent deficiency for each interval
    most_recurrent = {}

    for interval, deficiencies in deficiency_counts.items():
        most_recurrent[interval] = max(deficiencies, key=deficiencies.get)

    output = {}
    # Print the results
    # print("Most recurrent deficiencies for each 5ft interval:")
    for interval, deficiency in most_recurrent.items():
        output[interval] = {"deficiency": deficiency, "image": interval_images[interval][0]}
        # print(f"Interval {interval}ft - {interval+4}ft: {deficiency}")
        # print(f"Images for this interval: {interval_images[interval][0]}")
        # cv2.imwrite('s2.png', interval_images[interval][0])

    return output


def generateReport(deficiencies_log):
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
    new_image_size = 250  
    
    horizontal_offset = new_image_size // 2  

    for position_str, value in deficiencies_log.items():
        position = float(position_str)
        detection = value["deficiency"]
        img_to_overlay = value["image"]
        pos_ft = str(position_str) + "ft"
        cv2.putText(img, pos_ft, (center - 400, vertical_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(img, detection, (center - 300, vertical_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        img_thumbnail = cv2.resize(img_to_overlay, (new_image_size, new_image_size))
        img[y_offset - new_image_size // 2:y_offset + new_image_size // 2,
            center - horizontal_offset:center + horizontal_offset] = img_thumbnail

        vertical_pos += 300
        y_offset += 300


    cv2.imwrite('circle_with_pipe_and_deficiencies1.png', img)