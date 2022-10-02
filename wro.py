import time
import RPi.GPIO as GPIO
import cv2
import numpy as np
from picamera import PiCamera

camera = PiCamera()
img_path = '/home/wro/Desktop/img.jpg'

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

En = 40
In1 = 38
In2 = 36

trig = 3
echo = 5

servoPin = 33

pushBtn = 16

GPIO.setup(trig, GPIO.OUT)
GPIO.output(trig, False)
GPIO.setup(echo, GPIO.IN)
GPIO.setup(servoPin, GPIO.OUT)
GPIO.setup(En, GPIO.OUT)
GPIO.setup(In1, GPIO.OUT)
GPIO.setup(In2, GPIO.OUT)
GPIO.setup(pushBtn, GPIO.IN)

servo = GPIO.PWM(servoPin, 100)

driver = GPIO.PWM(En, 255)

turns_count = 0
direction = None


def getDistance():
    GPIO.output(trig, True)

    time.sleep(0.0001)
    GPIO.output(trig, False)

    startTime = time.time()
    endTime = time.time()

    while GPIO.input(echo) == 0:
        startTime = time.time()

    while GPIO.input(echo) == 1:
        endTime = time.time()

    elapsed = endTime - startTime

    dis = (elapsed * 34300) / 2

    return dis


def rotate(ang=0):
    if dir == 0:
        servo.ChangeDutyCycle(10)
    elif dir == -1:
        servo.ChangeDutyCycle(7)
    elif dir == 1:
        servo.ChangeDutyCycle(13.5)


def changeDirection(reverse=False):
    if reverse:
        GPIO.output(In1, False)
        GPIO.output(In2, True)
    else:
        GPIO.output(In1, True)
        GPIO.output(In2, False)


def crop_image(img):
    height = img.shape[0]
    width = img.shape[1]

    img = img[300:height, 0:width]

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_gray = np.array([0, 70, 0], dtype=np.uint8)
    upper_gray = np.array([179, 255, 60], dtype=np.uint8)

    mask_gray = cv2.inRange(hsv, lower_gray, upper_gray)

    # define kernel size
    kernel = np.ones((7, 7), np.uint8)
    # Remove unnecessary noise from mask
    mask_gray = cv2.morphologyEx(mask_gray, cv2.MORPH_CLOSE, kernel)
    mask_gray = cv2.morphologyEx(mask_gray, cv2.MORPH_OPEN, kernel)

    edged = cv2.Canny(mask_gray, 30, 100)
    lines = cv2.HoughLinesP(edged, 1, np.pi/60, 50,
                            maxLineGap=50, minLineLength=10)

    min_height = None
    if (lines is not None):
        for x in lines:
            for x1, y1, x2, y2 in x:
                if min_height is None:
                    min_height = min(y1, y2)
                else:
                    m = min(y1, y2)
                    min_height = min(min_height, m)
                cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 2)

    img = img[min_height:height, 0:width]
    return img


def define_direction():
    isCorner, corners_blue, corners_orange = is_corner()

    blue_middle = 0
    orange_middle = 0

    if corners_blue is not None and len(corners_blue) >= 30:
        for corner in corners_blue:
            x, y = corner.ravel()
            blue_middle = blue_middle + y

        blue_middle = blue_middle / len(corners_blue)

    if corners_orange is not None and len(corners_orange) >= 20:
        for corner in corners_orange:
            x, y = corner.ravel()
            orange_middle = orange_middle + y

        orange_middle = orange_middle / len(corners_orange)

    print('blue', blue_middle)
    print('otrange', orange_middle)
    if blue_middle == 0 and orange_middle == 0:
        return [isCorner, None]
    elif blue_middle > orange_middle:
        return [isCorner, -1]  # Left
    elif blue_middle < orange_middle:
        return [isCorner, 1]  # Right

    return [isCorner, None]


def is_corner():
    camera.capture(img_path)

    src = cv2.imread(img_path)

    img = cv2.rotate(src, cv2.ROTATE_180)

    img = crop_image(img)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([90, 0, 63])
    upper_blue = np.array([107, 255, 160])
    lower_orange = np.array([10, 50, 50])
    upper_orange = np.array([37, 205, 205])

    mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    # define kernel size
    kernel = np.ones((7, 7), np.uint8)

    # Remove unnecessary noise from mask
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)
    mask_orange = cv2.morphologyEx(mask_orange, cv2.MORPH_CLOSE, kernel)
    mask_orange = cv2.morphologyEx(mask_orange, cv2.MORPH_OPEN, kernel)

    corners_blue = cv2.goodFeaturesToTrack(mask_blue, 100, 0.01, 10)
    corners_orange = cv2.goodFeaturesToTrack(mask_orange, 100, 0.01, 10)

    blue_count = 0
    orange_count = 0
    if corners_blue is not None and len(corners_blue) >= 30:
        blue_count = len(corners_blue)
    if corners_orange is not None and len(corners_orange) >= 30:
        orange_count = len(corners_orange)

    return [blue_count != 0 or orange_count != 0, corners_blue, corners_orange]


def scan_road():
    camera.capture(img_path)


def getAngle():
    if direction == -1:
        return 15.3
    elif direction == 1:
        return 19.5
    return 17.3


def getUSDis(i):
    if i % 4 == 1:
        return 40
    return 70


if __name__ == '__main__':
    try:
        while True:
            if (not GPIO.input(pushBtn)):
                continue

            changeDirection(False)
            driver.start(0)
            servo.start(17.2)

            if (turns_count < 12):
                isCorner = False

                if direction is None:
                    isCorner, direction = define_direction()
                    direction = 1
                else:
                    isCorner = is_corner()[0]
                driver.ChangeDutyCycle(100)
                if isCorner:
                    while getDistance() > getUSDis(turns_count):
                        continue
                    servo.ChangeDutyCycle(getAngle())
                    if direction == -1:
                        time.sleep(2.5)
                    else:
                        time.sleep(0.8)

                    servo.ChangeDutyCycle(17.2)
                    time.sleep(1.7)
                    turns_count = turns_count + 1
                if getDistance() < 10:
                    changeDirection(True)
                    servo.ChangeDutyCycle(getAngle())
                    driver.ChangeDutyCycle(100)
                    time.sleep(0.6)
                    changeDirection(False)
                    servo.ChangeDutyCycle(17.2)
                    turns_count = turns_count - 1
                print('turns', turns_count)

            elif turns_count == 12:
                time.sleep(0.3)
                turns_count = 13
            else:
                driver.stop()
                servo.stop()

    except:
        GPIO.cleanup()
