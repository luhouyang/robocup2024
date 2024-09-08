import cv2
import numpy as np

# Load the MobileNetSSD model
# path to the prototxt file with text description of the network architecture
prototxt = "/home/lulu/Desktop/robocup2024/src/main_package/run/MobileNetSSD_deploy.prototxt"
# path to the .caffemodel file with learned network
caffe_model = "/home/lulu/Desktop/robocup2024/src/main_package/run/MobileNetSSD_deploy.caffemodel"

net = cv2.dnn.readNetFromCaffe(prototxt, caffe_model)

# Dictionary of class labels
classNames = {
    0: 'background',
    1: 'aeroplane',
    2: 'bicycle',
    3: 'bird',
    4: 'boat',
    5: 'bottle',
    6: 'bus',
    7: 'car',
    8: 'cat',
    9: 'chair',
    10: 'cow',
    11: 'diningtable',
    12: 'dog',
    13: 'horse',
    14: 'motorbike',
    15: 'person',
    16: 'pottedplant',
    17: 'sheep',
    18: 'sofa',
    19: 'train',
    20: 'tvmonitor'
}


def find_bounding_boxes(image):
    global person_box, first_person_detected

    height, width = image.shape[:2]

    # Construct a blob from the frame
    blob = cv2.dnn.blobFromImage(image,
                                 scalefactor=1 / 127.5,
                                 size=(300,
                                       300),
                                 mean=(127.5,
                                       127.5,
                                       127.5),
                                 swapRB=True,
                                 crop=False)
    net.setInput(blob)
    detections = net.forward()

    out_image = image.copy()
    new_person_box = None

    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > 0.5:
            class_id = int(detections[0, 0, i, 1])
            if class_id == 15:  # Class ID 15 is 'person'
                x_top_left = int(detections[0, 0, i, 3] * width)
                y_top_left = int(detections[0, 0, i, 4] * height)
                x_bottom_right = int(detections[0, 0, i, 5] * width)
                y_bottom_right = int(detections[0, 0, i, 6] * height)

                new_person_box = (x_top_left,
                                  y_top_left,
                                  x_bottom_right,
                                  y_bottom_right)
                break

    if new_person_box:
        x_top_left, y_top_left, x_bottom_right, y_bottom_right = new_person_box
        center_x = (x_top_left + x_bottom_right) / 2
        center_y = (y_top_left + y_bottom_right) / 2
        width = x_bottom_right - x_top_left
        height = y_bottom_right - y_top_left

        # Draw the bounding box on the output image (for visualization purposes)
        cv2.rectangle(out_image,
                      (x_top_left,
                       y_top_left),
                      (x_bottom_right,
                       y_bottom_right),
                      (0,
                       255,
                       0),
                      2)

        if center_x is not None:
            print(
                f"Center: ({center_x}, {center_y}), Size: ({width}, {height})")

        return center_x, center_y, width, height

    return None, None, None, None


#---------- Apply search window: returns the image
#-- return(image)
def apply_search_window(image, window_adim=[0.0, 0.0, 1.0, 1.0]):
    rows = image.shape[0]
    cols = image.shape[1]
    x_min_px = int(cols * window_adim[0] / 100)
    y_min_px = int(rows * window_adim[1] / 100)
    x_max_px = int(cols * window_adim[2] / 100)
    y_max_px = int(rows * window_adim[3] / 100)

    #--- Initialize the mask as a black image
    mask = np.zeros(image.shape, np.uint8)

    #--- Copy the pixels from the original image corresponding to the window
    mask[y_min_px:y_max_px,
         x_min_px:x_max_px] = image[y_min_px:y_max_px,
                                    x_min_px:x_max_px]

    #--- return the mask
    return (mask)


#---------- Draw search window: returns the image
#-- return(image)
def draw_window2(image,              #- Input image
                rect_px,        #- window in adimensional units
                color=(255,0,0),    #- line's color
                line=5,             #- line's thickness
               ):

    #-- Draw a rectangle from top left to bottom right corner

    return cv2.rectangle(image,
                         (rect_px[0],
                          rect_px[1]),
                         (rect_px[2],
                          rect_px[3]),
                         color,
                         line)


def convert_rect_perc_to_pixels(rect_perc, image):
    rows = image.shape[0]
    cols = image.shape[1]

    scale = [cols, rows, cols, rows]

    # x_min_px    = int(cols*window_adim[0])
    # y_min_px    = int(rows*window_adim[1])
    # x_max_px    = int(cols*window_adim[2])
    # y_max_px    = int(rows*window_adim[3])
    return [int(a * b / 100) for a, b in zip(rect_perc, scale)]


def normalise_keypoint(cv_image, kp):
    rows = float(cv_image.shape[0])
    cols = float(cv_image.shape[1])
    # print(rows, cols)
    center_x = 0.5 * cols
    center_y = 0.5 * rows
    # print(center_x)
    x = (kp.pt[0] - center_x) / (center_x)
    y = (kp.pt[1] - center_y) / (center_y)
    return cv2.KeyPoint(x, y, kp.size / cv_image.shape[1])


def create_tuning_window(initial_values):
    cv2.namedWindow("Tuning", 0)
    cv2.createTrackbar("x_min", "Tuning", initial_values['x_min'], 100, no_op)
    cv2.createTrackbar("x_max", "Tuning", initial_values['x_max'], 100, no_op)
    cv2.createTrackbar("y_min", "Tuning", initial_values['y_min'], 100, no_op)
    cv2.createTrackbar("y_max", "Tuning", initial_values['y_max'], 100, no_op)
    cv2.createTrackbar("h_min", "Tuning", initial_values['h_min'], 180, no_op)
    cv2.createTrackbar("h_max", "Tuning", initial_values['h_max'], 180, no_op)
    cv2.createTrackbar("s_min", "Tuning", initial_values['s_min'], 255, no_op)
    cv2.createTrackbar("s_max", "Tuning", initial_values['s_max'], 255, no_op)
    cv2.createTrackbar("v_min", "Tuning", initial_values['v_min'], 255, no_op)
    cv2.createTrackbar("v_max", "Tuning", initial_values['v_max'], 255, no_op)
    cv2.createTrackbar("sz_min",
                       "Tuning",
                       initial_values['sz_min'],
                       100,
                       no_op)
    cv2.createTrackbar("sz_max",
                       "Tuning",
                       initial_values['sz_max'],
                       100,
                       no_op)


def get_tuning_params():
    trackbar_names = [
        "x_min",
        "x_max",
        "y_min",
        "y_max",
        "h_min",
        "h_max",
        "s_min",
        "s_max",
        "v_min",
        "v_max",
        "sz_min",
        "sz_max"
    ]
    return {key: cv2.getTrackbarPos(key, "Tuning") for key in trackbar_names}


def wait_on_gui():
    cv2.waitKey(2)


def no_op(x):
    pass
