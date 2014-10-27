import cv2
import sys
import numpy as np
video_capture = cv2.VideoCapture(0)

def wholeLine():

    while True:
        # Capture frame-by-frame
        ret, frame = video_capture.read()

        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray,50,150,apertureSize = 3)

        lines = cv2.HoughLines(edges,1,np.pi/180,200)
        if lines is not None:
            for rho,theta in lines[0]:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))

                cv2.line(frame,(x1,y1),(x2,y2),(0,0,255),2)

        
        # Display the resulting frame
        cv2.imshow('Video', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything is done, release the capture
    video_capture.release()
    cv2.destroyAllWindows()

def smallLine():
    while True:
        # Capture frame-by-frame
        ret, frame = video_capture.read()

        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray,50,150,apertureSize = 3)
        minLineLength = 300
        maxLineGap = 10
        lines = cv2.HoughLinesP(edges,1,np.pi/180,100,minLineLength,maxLineGap)

        if lines == None:
            continue
        for x1,y1,x2,y2 in lines[0]:
            cv2.line(frame,(x1,y1),(x2,y2),(0,255,0),2)

            
            # Display the resulting frame
        cv2.imshow('Video', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # When everything is done, release the capture
    video_capture.release()
    cv2.destroyAllWindows()

def nothing(x):
    pass

def canny():
    cv2.namedWindow('image')
    # cv2.createTrackbar('pixelDif','image',0,100,nothing)
    cv2.createTrackbar('largeSize','image',0,1000,nothing)
    cv2.setTrackbarPos('largeSize','image',500)
    cv2.createTrackbar('lowH','image',0,255,nothing)
    cv2.setTrackbarPos('lowH','image',128)
    cv2.createTrackbar('lowS','image',0,255,nothing)
    cv2.setTrackbarPos('lowS','image',75)
    cv2.createTrackbar('lowV','image',0,255,nothing)
    cv2.setTrackbarPos('lowV','image',150)
    cv2.createTrackbar('highH','image',0,255,nothing)
    cv2.setTrackbarPos('highH','image',197)
    cv2.createTrackbar('highS','image',0,255,nothing)
    cv2.setTrackbarPos('highS','image',206)
    cv2.createTrackbar('highV','image',0,255,nothing)
    cv2.setTrackbarPos('highV','image',208)

    while True:
        # Capture frame-by-frame


        ret, frame = video_capture.read()

        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lowH = cv2.getTrackbarPos('lowH','image')
        lowS = cv2.getTrackbarPos('lowS','image')
        lowV = cv2.getTrackbarPos('lowV','image')
        highH = cv2.getTrackbarPos('highH','image')
        highS = cv2.getTrackbarPos('highS','image')
        highV = cv2.getTrackbarPos('highV','image')

        # define range of blue color in HSV
        lower_red1 = np.array([128,75,150])
        upper_red1 = np.array([197,206,208])

        lower_red2 = np.array([0,48,147])
        upper_red2 = np.array([4,206,255])

        lower_red3 = np.array([lowH,lowS,lowV])
        upper_red3 = np.array([highH,highS,highV])

        # Threshold the HSV image to get only blue colors
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        mask3 = cv2.inRange(hsv, lower_red3, upper_red3)


        mask12 = cv2.bitwise_or(mask1, mask2)

        mask = cv2.bitwise_or(mask12, mask3)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame,frame, mask= mask)

        gray = cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray,50,150,apertureSize = 3)
        contours, hierarchy = cv2.findContours(edges,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        largeContours = []

        # pixelDif = cv2.getTrackbarPos('pixelDif','image')
        largeSize = cv2.getTrackbarPos('largeSize','image')


        for cnt in contours:
            perimeter = cv2.arcLength(cnt,True)
            if perimeter > largeSize:
                largeContours.append(cnt)


        # smoothContours = []
        # for cnt in largeContours:
        #     error = 0.0
        #     if len(cnt) > pixelDif:
        #         for i in range(len(cnt)-pixelDif):
        #             midI = pixelDif/2
        #             dist = ((cnt[i][0][0]-cnt[i+pixelDif][0][0])**2 + (cnt[i][0][1]-cnt[pixelDif][0][1])**2 )**.5
        #             if dist < minErrorSize:
        #                 tempPT = ((cnt[i][0][0]+cnt[midI][0][0])/2,(cnt[i][0][1]+cnt[midI][0][1])/2)
        #                 dist = (tempPT[0]-cnt[midI][0][0])**2 + (tempPT[1]-cnt[midI][0][1])**2
        #                 error += dist
        #         averageError = error/len(cnt)
        #         print averageError
        #         if averageError < averageErrorSize:
        #             smoothContours.append(cnt)


        cv2.drawContours(res,largeContours,-1,(0,255,0),2)
            # Display the resulting frame
        cv2.imshow('Video', res)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # When everything is done, release the capture
    video_capture.release()
    cv2.destroyAllWindows()

canny()