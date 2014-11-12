import cv2
import numpy as np
from matplotlib import pyplot as plt
import Image
import time
timestart =  time.time()
print timestart

img = cv2.imread('sbob.jpg',0)
img2 = img.copy()
unscaled = Image.open('stop.jpg') 
scaleSize = 70 
scaled1 = unscaled.resize((scaleSize,scaleSize), Image.NEAREST)
scaled1.save("scaled1.png")
scaled2 = unscaled.resize((scaleSize+20,scaleSize+20), Image.NEAREST)
scaled2.save("scaled2.png")
scaled3 = unscaled.resize((scaleSize-20,scaleSize-20), Image.NEAREST)
scaled3.save("scaled3.png")
timestart =  time.time()


templates = [cv2.imread('scaled1.png',0), cv2.imread('scaled2.png',0), cv2.imread('scaled3.png',0)]


# All the 6 methods for comparison in a list
methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED']

for meth in methods:
    for template in templates:
        w, h = template.shape[::-1]

        img = img2.copy()
        method = eval(meth)

        # Apply template Matching
        res = cv2.matchTemplate(img,template,method)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

        # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
        if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
            top_left = min_loc
        else:
            top_left = max_loc
        bottom_right = (top_left[0] + w, top_left[1] + h)

        cv2.rectangle(img,top_left, bottom_right, 255, 2)
        print "this function took:" + str(time.time()- timestart) +"seconds"

        plt.subplot(121),plt.imshow(res,cmap = 'gray')
        plt.title('Matching Result'), plt.xticks([]), plt.yticks([])
        plt.subplot(122),plt.imshow(img,cmap = 'gray')
        plt.title('Detected Point'), plt.xticks([]), plt.yticks([])
        plt.suptitle(meth)
       
        plt.show()