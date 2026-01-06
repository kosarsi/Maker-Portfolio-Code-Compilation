import cv2
import numpy as np
import math

testVar = 0

def incrementTestVar():
    global testVar
    testVar = testVar + 1
    if testVar == 100:
        print("test")
    if testVar >= 200:
        print("print")
        testVar = 0

def drawDecorations(image):
    cv2.putText(image,
                'Limelight python script!',
                (0, 230),
                cv2.FONT_HERSHEY_SIMPLEX,
                .5, (0, 255, 0), 1, cv2.LINE_AA)

def runPipeline(image, llrobot):
    minArea = 2000
    aspectRatioTolerance = 0.2

    target_aspect_ratios = [7/3, 3/7]

    # Perspective transform
    pts1 = np.float32([[180, 65], [510, 65], [110, 410], [580, 410]])
    pts2 = np.float32([[80, 0], [560, 0], [80, 480], [560, 480]])
    M = cv2.getPerspectiveTransform(pts1, pts2)
    image = cv2.warpPerspective(image, M, (640, 480))

    # HSV thresholding
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    img_threshold = cv2.inRange(img_hsv, (20, 90, 90), (30, 255, 255))

    # Find contours
    contours, _ = cv2.findContours(img_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    scoredContours = []
    for cnt in contours:
        if cv2.contourArea(cnt) >= minArea:
            rect = cv2.minAreaRect(cnt)
            width, height = rect[1]
            if width > height:
                width, height = height, width

            if height == 0:  # Prevent division by zero
                continue

            aspectRatio = width / float(height)
            aspect_diff = min(abs(aspectRatio - r) for r in target_aspect_ratios)

            if aspect_diff <= aspectRatioTolerance:
                scoredContours.append((aspect_diff, cnt))

    # Sort by closest aspect ratio difference (lower is better)
    scoredContours = sorted(scoredContours, key=lambda x: x[0])
    topContours = [x[1] for x in scoredContours[:3]]

    angles = [0, 0, 0]
    positions = [0, 0, 0]
    number = 0

    for contour in topContours:
        rect = cv2.minAreaRect(contour)
        angle = rect[2]
        pos = rect[0]
        x, y = pos
        x = math.floor(x)
        y = math.floor(y)

        positions[number] = x + y / 1000
        angles[number] = angle
        if rect[1][0] < rect[1][1]:
            angles[number] += 90
        angles[number] = math.floor(angles[number])
        number += 1

        cv2.drawContours(image, [contour], -1, (0, 255, 0), 2)

    largestContour = topContours[0] if topContours else np.array([[]])

    llpython = [1, len(topContours), positions[0], angles[0], positions[1], angles[1], positions[2], angles[2]]
    incrementTestVar()
    return topContours, image, llpython