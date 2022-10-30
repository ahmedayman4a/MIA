from math import pi
import cv2
import numpy as np
#import imutils

def ResizeWithAspectRatio(image, width=None, height=None, inter=cv2.INTER_AREA):
    dim = None
    (h, w) = image.shape[:2]

    if width is None and height is None:
        return image
    if width is None and height < h:
        r = height / float(h)
        dim = (int(w * r), height)
    elif height is None and width < w:
        r = width / float(w)
        dim = (width, int(h * r))

    return cv2.resize(image, dim, interpolation=inter)


 
# Read the original image
#img = cv2.imread('./robocon22/images/normal/lagoris/lagori_002.jpg') 
img = cv2.imread('./robocon22/images/normal/balls/ball_040.jpg')
# Display original image
resize = ResizeWithAspectRatio(img, height=756)
cv2.imshow('Original', resize)
# cv2.waitKey(0)
 
# Convert to graycsale
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# Blur the image for better edge detection
img_blur = cv2.GaussianBlur(img_gray, (11,11), 0) 
# clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
# equalized = clahe.apply(gray)
BLUE_MIN = np.array([110, 50, 50],np.uint8)
BLUE_MAX = np.array([125, 255, 255],np.uint8)

DARK_MIN = np.array([100, 80, 20],np.uint8)
DARK_MAX = np.array([120, 255, 255],np.uint8)

WHITE_MIN = np.array([20, 0, 170],np.uint8)
WHITE_MAX = np.array([120, 255, 255],np.uint8)
# convert from RGB color-space to YCrCb
ycrcb_img = cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)

# equalize the histogram of the Y channel
#ycrcb_img[:, :, 0] = cv2.equalizeHist(ycrcb_img[:, :, 0])
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(10, 10))
ycrcb_img[:, :, 0] = clahe.apply(ycrcb_img[:, :, 0])
# convert back to RGB color-space from YCrCb
equalized_img = cv2.cvtColor(ycrcb_img, cv2.COLOR_YCrCb2BGR)
resize = ResizeWithAspectRatio(equalized_img, height=756)
cv2.imshow('equalized', resize)
hsv_img = cv2.cvtColor(equalized_img,cv2.COLOR_BGR2HSV)

blue_threshed = cv2.inRange(hsv_img, BLUE_MIN, BLUE_MAX)
resize = ResizeWithAspectRatio(blue_threshed, height=756)
# cv2.imshow('blue', resize)

dark_threshed = cv2.inRange(hsv_img, DARK_MIN, DARK_MAX)
resize = ResizeWithAspectRatio(dark_threshed, height=756)
cv2.imshow('dark', resize)

# white_threshed = cv2.inRange(hsv_img, WHITE_MIN, WHITE_MAX)
# resize = ResizeWithAspectRatio(white_threshed, height=756)
# cv2.imshow('white', resize)

# Canny Edge Detection
# edges = cv2.Canny(image=frame_threshed, threshold1=240, threshold2=250) # Canny Edge Detection
# # Display Canny Edge Detection Image
# resize = ResizeWithAspectRatio(edges, height=756)
# cv2.imshow('tight', resize)

# thresh = cv2.threshold(edges, 200, 255,
# 	cv2.THRESH_BINARY_INV)[1]
# resize = ResizeWithAspectRatio(thresh, height=756)
# cv2.imshow('thresh', resize)

# cnts = cv2.findContours(thresh.copy(), cv2.RETR_LIST,
# 	cv2.CHAIN_APPROX_SIMPLE)
# cnts = imutils.grab_contours(cnts)
# c = max(cnts, key=cv2.contourArea)

# # draw the shape of the contour on the output image, compute the
# # bounding box, and display the number of points in the contour
# output = img.copy()
# cv2.drawContours(output, [c], -1, (0, 255, 0), 3)
# (x, y, w, h) = cv2.boundingRect(c)
# # text = "original, num_pts={}".format(len(c))
# # cv2.putText(output, text, (x, y - 15), cv2.FONT_HERSHEY_SIMPLEX,
# # 	1.5, (0, 255, 0), 2)

# cv2.rectangle(output, (x,y), (x+w,y+h), (0, 0, 255), 2)
# define a (3, 3) structuring element
# kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
# kernel2 = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

# # apply the dilation operation to the edged image

# dilate = cv2.dilate(frame_threshed, kernel, iterations=1)
# erode = cv2.erode(dilate, kernel2, iterations=1)
# resize = ResizeWithAspectRatio(erode, height=756)
# cv2.imshow("eroded", resize)
# resize = ResizeWithAspectRatio(dilate, height=756)
# cv2.imshow("dilated", resize)
#kernel = np.ones((3,3),np.uint8)



kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
#morph = cv2.morphologyEx(frame_threshed, cv2.MORPH_GRADIENT, kernel)
morph = cv2.morphologyEx(blue_threshed, cv2.MORPH_OPEN, kernel)
# morph = cv2.GaussianBlur(morph, (11,11), 0) 
resize = ResizeWithAspectRatio(morph, height=756)
# cv2.imshow("morph", resize)


kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

# apply the dilation operation to the edged image


# erode = cv2.erode(dark_threshed, kernel, iterations=2)
# dilate = cv2.dilate(erode, kernel, iterations=4)
# resize = ResizeWithAspectRatio(erode, height=756)
# cv2.imshow("eroded", resize)
# resize = ResizeWithAspectRatio(dilate, height=756)
# cv2.imshow("dilated", resize)

kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
#morph = cv2.morphologyEx(frame_threshed, cv2.MORPH_GRADIENT, kernel)
dark_morph = cv2.morphologyEx(dark_threshed, cv2.MORPH_OPEN, kernel)
# morph = cv2.GaussianBlur(morph, (11,11), 0) 
resize = ResizeWithAspectRatio(dark_morph, height=756)
cv2.imshow("dark morph", resize)

# kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
# #morph = cv2.morphologyEx(frame_threshed, cv2.MORPH_GRADIENT, kernel)
# morph = cv2.morphologyEx(dark_threshed, cv2.MORPH_OPEN, kernel)
# morph = cv2.GaussianBlur(morph, (5,5), 0) 
# resize = ResizeWithAspectRatio(morph, height=756)
# cv2.imshow("morph", resize)

# kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
# #morph = cv2.morphologyEx(frame_threshed, cv2.MORPH_GRADIENT, kernel)
# morph_white = cv2.morphologyEx(white_threshed, cv2.MORPH_OPEN, kernel)
# morph_white = cv2.GaussianBlur(morph_white, (17,17), 0) 
# # morph = cv2.GaussianBlur(morph, (11,11), 0) 
# resize = ResizeWithAspectRatio(morph_white, height=756)
# cv2.imshow("morph_white", resize)



contours, _ = cv2.findContours(dark_morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
output = img.copy()
# draw the contours on a copy of the original image
# approx = cv2.approxPolyDP(cnt,0.01*cv.arcLength(cnt,True),True)
for cnt in contours:
    area = cv2.contourArea(cnt)
    
    if area > img.shape[0] * img.shape[1] // 2:
      continue  
  
    approx = cv2.approxPolyDP(cnt,0.05*cv2.arcLength(cnt,True),True)
    
    
    if len(approx)==4 and cv2.contourArea(cnt) > 1000:
        x,y,w,h = cv2.boundingRect(approx)

        #width:height
        aspectRatio = float(w)/h
        (x, y, w, h) = cv2.boundingRect(cnt)
        cv2.rectangle(output,(x,y),(x+w,y+h),(0,0,255),5)
            
# resize = ResizeWithAspectRatio(output, height=756)
# cv2.imshow("Detected Rectangle", resize)

 

# Apply Hough transform on the blurred image.
detected_circles = cv2.HoughCircles(morph, 
                   cv2.HOUGH_GRADIENT, 2, 3000, param1 = 200,
               param2 = 60, minRadius = 0, maxRadius = 300)
#output = img.copy()
# Draw circles that are detected.
if detected_circles is not None:
    # convert the (x, y) coordinates and radius of the circles to integers
    circles = np.round(detected_circles[0, :]).astype("int")
    # loop over the (x, y) coordinates and radius of the circles
    for (x, y, r) in circles:
        # draw the circle in the output image, then draw a rectangle
        # corresponding to the center of the circle
        cv2.rectangle(output,(x-r,y-r),(x+r,y+r),(0, 255, 0), 5)
        # circle_img = np.zeros((img.shape[0],img.shape[1]), np.uint8)
        # cv2.circle(circle_img,(x,y),r,(255,255,255),-1)
        # datos_rgb = cv2.mean(hsv_img, mask=circle_img)
        # cv2.circle(output, (x, y), r, (0, 255, 0), 4)
        # cv2.rectangle(output, (x - 1, y - 1), (x + 1, y + 1), (0, 128, 255), -1)
        
resize = ResizeWithAspectRatio(output, height=756)
cv2.imshow("Detected", resize)



cv2.waitKey(0)
cv2.destroyAllWindows()