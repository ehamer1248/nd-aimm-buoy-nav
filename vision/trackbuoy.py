import cv2
import numpy as np

# DEBUG OPTIONS
debug_drawEnclosingCircle = False


# CONSTANTS
RED       = np.uint8([[[255,0,0]]])
RED_HSV   = cv2.cvtColor(RED, cv2.COLOR_BGR2HSV)
RED_LOWER1 = np.array([0, 100, 20])
RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([160, 100, 20])
RED_UPPER2 = np.array([180, 255, 255])

# FUNCTIONS
def get_hsv(event, x, y, flags, param):
    global ix,iy,show_hsv

    if event == cv2.EVENT_LBUTTONDOWN:
        print('Updating shown HSV')
        ix,iy,show_hsv = x,y,True

def setup_frame():
    cv2.namedWindow('frame')
    cv2.setMouseCallback('frame', get_hsv)

# read frame
def read_frame(frame):
    ix,iy,show_hsv = 0,0,False
    tar_h,tar_s,tar_v = 0,0,0
    while True:
        
        # convert image to HSV and give strong blur
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frame_hsv = cv2.GaussianBlur(frame_hsv, (11,11), 5)

        # make red mask with HSV range
        lower_mask = cv2.inRange(frame_hsv, RED_LOWER1, RED_UPPER1)
        upper_mask = cv2.inRange(frame_hsv, RED_LOWER2, RED_UPPER2)

        mask = lower_mask + upper_mask

        # compute final image
        res = cv2.bitwise_and(frame, frame, mask=mask)

        # find contours in mask
        contours,hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            max_contour = max(contours, key=cv2.contourArea)
            cv2.drawContours(frame, [max_contour], -1, (0,255,0), 3)

            if debug_drawEnclosingCircle:
                (x,y),radius = cv2.minEnclosingCircle(max_contour)
                center = (int(x), int(y))
                radius = int(radius)

                cv2.circle(frame, center, radius, (255,0,0),2)

        # show HSV at target location
        if show_hsv:
            tar_h = frame.item(iy,ix,0)
            tar_s = frame.item(iy,ix,1)
            tar_v = frame.item(iy,ix,2)
            show_hsv = False

        cv2.putText(frame, f'H:{tar_h} | S:{tar_s} | V:{tar_v}', (20,20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0))

        # show images and check for break
        cv2.imshow('frame', frame)
        cv2.imshow('mask', mask)
        cv2.imshow('res', res)


def close_frame():
    k = cv2.waitKey(1)
    if k == 27:
        cv2.destroyAllWindows()