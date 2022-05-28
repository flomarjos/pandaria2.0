from sre_constants import CATEGORY_UNI_NOT_LINEBREAK
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.ticker import NullLocator

def computeDepth2Coordinates(depth, pixel_x, pixel_y, camera_intrinsics):
    """
	Convert the depth and image point information to metric coordinates
	Parameters:
	-----------
	depth 	 	 	     : double
						   The depth value of the image point
	pixel_x 	  	 	 : double
						   The x value of the image coordinate
	pixel_y 	  	 	 : double
							The y value of the image coordinate
	camera_intrinsics : The intrinsic values of the imager in whose coordinate system the depth_frame is computed
	Return:
	----------
	X : double
		The x value in meters
	Y : double
		The y value in meters
	Z : double
		The z value in meters
	"""
    #print(camera_intrinsics)
    fx = 611.614013671875       #camera_intrinsics.K[0]
    fy = 610.2874755859375      #camera_intrinsics.K[4]
    cx = 320.3898620605469      #camera_intrinsics.K[2]
    cy = 250.4097137451172      #camera_intrinsics.K[5]
    X = (pixel_x - cx)/fx*depth * 0.001
    Y = (pixel_y - cy)/fy*depth * 0.001

    return [X, Y, depth*0.001]
    
def preprocessImage(img):
    """
    Preparation of image for feature detection
    -----------
    Args:
          img               : cv2 image
    
    Return:
          imgThres          : cv2 image    

    """
    imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    imgBlur = cv2.GaussianBlur(imgGray,(5,5),1)
    imgCanny = cv2.Canny(imgBlur,110,170)
    kernel = np.ones((5,5))
    imgDilat = cv2.dilate(imgCanny,kernel,iterations=2)
    imgThres = cv2.erode(imgDilat,kernel,iterations=1)
    return imgThres

def computeMeanOrange(img):
    """ 
    Computes the coordinates of the esp32 controller after masking its area with
    color threshold operations
    Args:
         img         : cv2 image
    Return:
         maskCenter  : [x, y] pixel coordinates of esp32 moment
    """

    cv_img_proc = preprocessImage(cv_img)
    _, contours, _ = cv2.findContours(cv_img_proc,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.1*peri, True)
        objcorners = len(approx)    
        #                     750<peri<950
                            #1000<peri<1200
        if objcorners==4 and  peri <300:#<2500:
            a = np.squeeze(approx)
            return a[:,0]



    # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # #lower and upper bounds
    # #lower = np.array([0., 140., 150.])
    # #upper = np.array([100., 255., 255.])
    # lower = np.array([0., 150., 170.])
    # upper = np.array([90., 255., 255.])

    # mask = cv2.inRange(hsv, lower, upper)
    # sth = np.asarray(np.where(mask==255))
    # if sth.size != 0:
    #     maskCenter = np.mean(np.where(mask==255),axis=1,dtype='int')
    #     return np.asarray([maskCenter[1],maskCenter[0]]) #returning in cv coordinates
    # else:
    #     return None
        
    

def computeYawRotation(idx, cornerPoints):
    """
    Computation of the rotation around Z of the reference frame that locates in the corner of the box
    Args:
         idx          : index in "cornerPoints" that represents the origin of reference frame
         cornerPoints : Array of the corner points of the box.
    Return:
         yawAngle     : rotation angle around Z axis
    """
    nearestDistance= 9999999999
    nearestPoint = np.zeros(2)

    for i in range(4):
        if idx != i:
            norm = np.linalg.norm(cornerPoints[i] - cornerPoints[idx])
            if norm < nearestDistance:
                nearestDistance = norm
                nearestPoint = np.squeeze(cornerPoints[i])
                print("nearestPoint: ", nearestPoint)
                print("idx: ", idx)
                print("i: ", i)
                print("nearestDistance", nearestDistance)
                print("cornerPoints: ", cornerPoints)
    new_Ax = np.squeeze(nearestPoint - cornerPoints[idx])
    new_Ax = new_Ax/np.linalg.norm(new_Ax)
    yawAngle = np.arctan2(new_Ax[1] , new_Ax[0])
    print("yawAngle", yawAngle*180/3.14159)
    return yawAngle, nearestPoint

def refinedOrientation(prepImg):
    yawAngles = np.array([])

    # bottom
    bottom = np.array( [[(0,prepImg.shape[0]//2),(0,prepImg.shape[0]),
            (prepImg.shape[1],prepImg.shape[0]),(prepImg.shape[1],prepImg.shape[0]//2)]], dtype=np.int32 )
    imgBottom = omitRegion(prepImg,bottom)
    lines = cv2.HoughLinesP(imgBottom,rho=1,theta=np.pi/180,threshold=255,lines=np.array([]),minLineLength=350,maxLineGap=1)
    if lines is not None:
        for line in lines:
            for x1,y1,x2,y2 in line:
                angle = np.arctan2((y2-y1),(x2-x1))-(np.pi/2)
                yawAngles = np.append(yawAngles,angle)

    # top
    top = np.array( [[(0,0),(0,prepImg.shape[0]//2),
            (prepImg.shape[1],prepImg.shape[0]//2),(prepImg.shape[1],0)]], dtype=np.int32 )
    imgTop = omitRegion(prepImg,top)
    lines = cv2.HoughLinesP(imgTop,rho=1,theta=np.pi/180,threshold=255,lines=np.array([]),minLineLength=350,maxLineGap=1)
    if lines is not None:
        for line in lines:
            for x1,y1,x2,y2 in line:
                angle = np.arctan2((y2-y1),(x2-x1))-(np.pi/2)
                yawAngles = np.append(yawAngles,angle)

    # left
    left = np.array( [[(0,0),(0,prepImg.shape[0]),
            (prepImg.shape[1]//4,prepImg.shape[0]),(prepImg.shape[1]//4,0)]], dtype=np.int32 )
    imgLeft = omitRegion(prepImg,left)
    lines = cv2.HoughLinesP(imgLeft,rho=1,theta=np.pi/180,threshold=255,lines=np.array([]),minLineLength=200,maxLineGap=1)
    if lines is not None:
        for line in lines:
            for x1,y1,x2,y2 in line:
                if y1>y2:   angle = np.arctan2((y2-y1),(x2-x1))
                else:       angle = np.arctan2((y1-y2),(x1-x2))
                yawAngles = np.append(yawAngles,angle)

    # right
    right = np.array( [[(prepImg.shape[1]//3*2,0),(prepImg.shape[1]//3*2,prepImg.shape[0]),
            (prepImg.shape[1],prepImg.shape[0]),(prepImg.shape[1],0)]], dtype=np.int32 )
    imgRight = omitRegion(prepImg,right)
    lines = cv2.HoughLinesP(imgRight,rho=1,theta=np.pi/180,threshold=255,lines=np.array([]),minLineLength=200,maxLineGap=1)
    if lines is not None:
        for line in lines:
            for x1,y1,x2,y2 in line:
                if y1>y2:   angle = np.arctan2((y2-y1),(x2-x1))
                else:       angle = np.arctan2((y1-y2),(x1-x2))
                yawAngles = np.append(yawAngles,angle)

    yawAngle = np.median(yawAngles)

    return yawAngle

def omitRegion(img,vertices):
    omitMask = np.zeros_like(img)
    if len(img.shape) == 2:
        cv2.fillPoly(omitMask,vertices,255)
    else:
        cv2.fillPoly(omitMask,vertices,(255,)*img.shape[2])
    return cv2.bitwise_and(img,omitMask)