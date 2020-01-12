import numpy as np
import cv2 as cv

# get the camera data to unwarp and undistort images
camera_matrix = np.loadtxt("cameraMatrix.txt", delimiter=",")
distortion = np.loadtxt("cameraDistortion.txt", delimiter=",")

print(camera_matrix)
print(distortion)


cap = cv.VideoCapture(2)
_, frame = cap.read()
h,  w = frame.shape[:2]
new_camera_matrix, roi = cv.getOptimalNewCameraMatrix(camera_matrix, distortion, (w, h), 1, (w, h))

kernel = np.ones((5,5),np.float32)/25
kernel2 = np.ones((5,5),np.uint8)
x, y, w, h = 100, 100, 200, 300 # simply hardcoded the values
track_window = (x, y, w, h)
term_crit = ( cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 1)

# params for ShiTomasi corner detection
feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )
# Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (15,15),
                  maxLevel = 2,
                  criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))
# Create some random colors
color = np.random.randint(0,255,(100,3))

try:
    counter = 0
    while(1):
        # Take each frame
        _, frame = cap.read()
        # undistort
        dst = cv.undistort(frame, camera_matrix, distortion, None, new_camera_matrix)
        # Convert BGR to HSV
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        # define range of blue color in HSV
        lower_blue = np.array([0,50,50])
        upper_blue = np.array([5,255,255])
        # Threshold the HSV image to get only blue colors
        mask = cv.inRange(hsv, lower_blue, upper_blue)
        # Bitwise-AND mask and original image
        res = cv.bitwise_and(frame,frame, mask= mask)
        gray = cv.cvtColor(res, cv.COLOR_BGR2GRAY)
        blur = cv.GaussianBlur(gray, (5, 5), 0)
        opening = cv.morphologyEx(blur, cv.MORPH_OPEN, kernel2)
        closed = cv.morphologyEx(opening, cv.MORPH_CLOSE, kernel2)

        # # apply camshift/meanshift to get the new location
        # ret, track_window = cv.CamShift(closed, track_window, term_crit)
        # # Draw it on image
        # #meanshift
        # #x,y,w,h = track_window
        # #img2 = cv.rectangle(frame, (x,y), (x+w,y+h), 255,2)
        #
        # #camshift
        # pts = cv.boxPoints(ret)
        # pts = np.int0(pts)
        # print(pts)
        # img2 = cv.polylines(frame,[pts],True, 255,2)
        # cv.imshow('img2',img2)


        if counter == 0:
            p0 = cv.goodFeaturesToTrack(closed, mask=None, **feature_params)
            # Create a mask image for drawing purposes
            mask2 = np.zeros_like(frame)
            old_image = closed
        else:
            # calculate optical flow
            p1, st, err = cv.calcOpticalFlowPyrLK(old_image, closed, p0, None, **lk_params)
            # Select good points
            good_new = p1[st == 1]
            good_old = p0[st == 1]
            # draw the tracks
            for i, (new, old) in enumerate(zip(good_new, good_old)):
                a, b = new.ravel()
                c, d = old.ravel()
                mask2 = cv.line(mask2, (a, b), (c, d), color[i].tolist(), 2)
                frame = cv.circle(frame, (a, b), 5, color[i].tolist(), -1)
            img = cv.add(frame, mask2)
            old_image = closed.copy()
            p0 = good_new.reshape(-1, 1, 2)

            cv.imshow('mask', img)

        counter += 1

        cv.imshow('frame',frame)
        cv.imshow('res',res)
        cv.imshow('opening', closed)
        k = cv.waitKey(5) & 0xFF
        if k == 27:
            break
    # When everything done, release the capture
    cap.release()
    cv.destroyAllWindows()
except KeyboardInterrupt:
    # When everything done, release the capture
    cap.release()
    cv.destroyAllWindows()