import cv2
import time
import numpy as np

# Open the default camera
cam = cv2.VideoCapture('videos_rasp/video_10.mp4')

def get_limits(color):
    c = np.uint8([[color]])  # BGR values
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
    hue = hsvC[0][0][0]  # Get the hue value

    if hue >= 165:  # Upper limit for divided red hue
        lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
        upperLimit = np.array([180, 255, 255], dtype=np.uint8)
    elif hue <= 15:
        lowerLimit = np.array([0, 100, 100], dtype=np.uint8)
        upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)
    else:
        lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
        upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)

    return lowerLimit, upperLimit

def merge_close_boxes(bounding_boxes, distance_threshold=30):
    """
    Merges bounding boxes.
    """
    merged_boxes = []
    while bounding_boxes:
        box = bounding_boxes.pop(0)
        x, y, w, h = box

        boxes_to_merge = [box]
        for other_box in bounding_boxes:
            ox, oy, ow, oh = other_box
            if (x < ox + ow + distance_threshold and x + w + distance_threshold > ox and
                y < oy + oh + distance_threshold and y + h + distance_threshold > oy):
                boxes_to_merge.append(other_box)

        bounding_boxes = [b for b in bounding_boxes if b not in boxes_to_merge]

        min_x = min([b[0] for b in boxes_to_merge])
        min_y = min([b[1] for b in boxes_to_merge])
        max_x = max([b[0] + b[2] for b in boxes_to_merge])
        max_y = max([b[1] + b[3] for b in boxes_to_merge])
        merged_box = (min_x, min_y, max_x - min_x, max_y - min_y)

        merged_boxes.append(merged_box)

    return merged_boxes

def main():
    desired_fps = 30
    frame_duration = 1 / desired_fps  # Time for each frame
    i = 0

    # Get the default frame width and height
    frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('output.mp4', fourcc, 30.0, (frame_width, frame_height))



    while True:
        start_time = time.time()
        ret, frame = cam.read()
        
        if not ret:
            continue

        hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lowerLimit, upperLimit = get_limits(color=(0, 0, 255))

        mask = cv2.inRange(hsvImage, lowerLimit, upperLimit)

        # Dilate
        kernel = np.ones((10, 10), np.uint8)
        dilated_image = cv2.dilate(mask, kernel, iterations=1)

        # Find contours
        contours_dilated, _ = cv2.findContours(dilated_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Get bounding boxes
        initial_boxes = [cv2.boundingRect(contour) for contour in contours_dilated]

        # Merge bboxes
        merged_boxes = merge_close_boxes(initial_boxes, distance_threshold=30)

        # Draw
        for (x, y, w, h) in merged_boxes:

            if (w*h > 1500):
                # print(f"S = {w*h}")
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (int(x+w/2), int(y+h/2)), 4, (255, 255, 0), -1)

        cv2.imshow('Camera RGB', frame)

        # Write video
        out.write(frame)

        # cv2.imwrite(f"./data/{i}.png", mask)
        i += 1

        # Control FPS
        elapsed_time = time.time() - start_time
        delay = max(1, int((frame_duration - elapsed_time) * 1000))  # Delay to maintain FPS

        if cv2.waitKey(delay) == ord('q'):
            break

if __name__ == "__main__":
    main()
