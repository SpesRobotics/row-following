import cv2

# Load an image in grayscale
gray_image = cv2.imread('/home/milos/row-following/ros2_ws/Dataset/MaskedCilantroOm_4.png', cv2.IMREAD_GRAYSCALE)

# Get image dimensions
height, width = gray_image.shape  # This will correctly unpack two values since gray_image is guaranteed to be grayscale

# Convert to a binary image and invert it (if needed based on your image colors)
_, binary_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY_INV)

# Find all contours
contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Sort contours by area, descending
contours = sorted(contours, key=cv2.contourArea, reverse=True)

# Create a color image for visualization
color_image = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)

# Check if there are at least three contours, else take as many as available
max_contours = min(3, len(contours))
bounding_boxes = []

# Draw the three largest contours
for i in range(max_contours):
    contour = contours[i]
    x, y, w, h = cv2.boundingRect(contour)
    cv2.rectangle(color_image, (x, y), (x+w, y+h), (0, 255, 0), 2)  # Draw bounding box in green
    # Optionally draw the contour
    # cv2.drawContours(color_image, [contour], -1, (255, 0, 0), 2)  # Draw contour in blue
    bounding_boxes.append((x, y, x+w, y+h))

bounding_boxes.sort(key=lambda box: box[0]) #Sort the Rectangles from left to right

for i in range(len(bounding_boxes) - 1):  # We draw rectangles between i and i+1
    # Get the current box and the next one
    curr_box = bounding_boxes[i]
    next_box = bounding_boxes[i+1]

    # Calculate intermediate top left and bottom right points
    top_left_x = curr_box[2]  # right side of the current box
    bottom_right_x = next_box[0]  # left side of the next box
    top_left_y = min(curr_box[1], next_box[1])  # minimum of the top y-values
    bottom_right_y = max(curr_box[1] + (curr_box[3] - curr_box[1]), next_box[1] + (next_box[3] - next_box[1]))  # maximum of the bottom y-values

    # Draw a rectangle using these intermediate points
    if bottom_right_x > top_left_x:  # Ensure there is a space to draw a rectangle
        cv2.rectangle(color_image, (top_left_x, top_left_y), (bottom_right_x, bottom_right_y), (255, 0, 0), 2)  # Blue rectangle

# Show the results
cv2.imshow('Largest Contours', color_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
