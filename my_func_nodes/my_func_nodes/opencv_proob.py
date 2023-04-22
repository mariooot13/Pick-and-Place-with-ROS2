import cv2 as cv
import numpy as np
import torch
import message_filters
import rospy
import sys
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rospkg import RosPack # get abs path
from os import path # get home path
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import *
from pyquaternion import Quaternion as PyQuaternion

# Global variables
path_yolo = path.join(path.expanduser('~'), 'yolov5')
path_vision = RosPack().get_path('vision')
path_weigths = path.join(path_vision, 'weigths')

cam_point = (-0.44, -0.5, 1.58)
height_tavolo = 0.74
dist_tavolo = None
origin = None
model = None
model_orientation = None

legoClasses = ['X1-Y1-Z2', 'X1-Y2-Z1', 'X1-Y2-Z2', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z2-TWINFILLET', 'X1-Y3-Z2', 'X1-Y3-Z2-FILLET', 'X1-Y4-Z1', 'X1-Y4-Z2', 'X2-Y2-Z2', 'X2-Y2-Z2-FILLET']

argv = sys.argv
a_show = '-show' in argv

# Utility Functions

def get_dist_tavolo(depth, hsv, img_draw):
    global dist_tavolo

    #color = (120,1,190)
    #mask = get_lego_mask(color, hsv, (5, 5, 5))
    #dist_tavolo = depth[mask].max()
    #if dist_tavolo > 1: dist_tavolo -= height_tavolo
    dist_tavolo = np.nanmax(depth)
    
    
    ##calculate the maximum depth value from an array called "depth" and assigns it to the variable "dist_tavolo"

def get_origin(img):
    global origin
    origin = np.array(img.shape[1::-1]) // 2
    
    """that takes an image as input and calculates its center point as a numpy array, which is stored in the global variable "origin".

The "img.shape" method returns the dimensions of the input image as a tuple (height, width, channels). The slice "img.shape[1::-1]" selects the width and height dimensions in reverse order (i.e., (width, height)), which is necessary because the numpy array is indexed in row-major order."""

def get_lego_distance(depth):
    return depth.min()

def get_lego_color(center, rgb):
    return rgb[center].tolist()
    
    """ two inputs: "center", which is a tuple of (x,y) coordinates representing the center of a LEGO brick on an image, and "rgb", which is a numpy array of RGB values for each pixel in the image. The function returns the RGB color value of the pixel at the center of the LEGO brick as a list.

The function retrieves the RGB values of the pixel at the center of the LEGO brick by indexing into the "rgb" numpy array using the "center" tuple as the index. The ".tolist()" method is then used to convert the resulting numpy array into a Python list."""

def get_lego_mask(color, hsv, toll = (20, 20, 255)):
    thresh = np.array(color)
    mintoll = thresh - np.array([toll[0], toll[1], min(thresh[2]-1, toll[2])])
    maxtoll = thresh + np.array(toll)
    return cv.inRange(hsv, mintoll, maxtoll)
    
"""This is a Python function called "get_lego_mask" that takes three inputs: "color", which is a list of three integers representing the HSV color values of a LEGO brick; "hsv", which is a numpy array of HSV values for each pixel in an image; and "toll", which is an optional tuple of three integers representing the tolerance for color detection in the HSV color space. The function returns a binary mask that indicates which pixels in the image match the specified LEGO color within the specified tolerance.

The function calculates a lower and upper bound for the HSV values that correspond to the specified LEGO color within the specified tolerance. The "mintoll" variable represents the lower bound, which is obtained by subtracting the tolerance values from the specified color values. The "maxtoll" variable represents the upper bound, which is obtained by adding the tolerance values to the specified color values. The minimum of the third value in "thresh" and "toll[2]" is used to avoid negative values.

The numpy function "np.array()" is used to convert the "color" and "toll" input values into numpy arrays. The resulting numpy arrays are then used to perform element-wise addition and subtraction with other numpy arrays.

Finally, the function uses the OpenCV function "cv.inRange()" to create a binary mask that indicates which pixels in the "hsv" numpy array fall within the specified range of HSV values. Pixels within the range are set to 255 in the mask, while pixels outside the range are set to 0. The resulting binary mask can be used to isolate the pixels in the image that correspond to the specified LEGO color within the specified tolerance.

In the context of LEGO robotics, this function could be used to create a mask that identifies the pixels in an image that correspond to LEGO bricks of a certain color. This mask could then be used for further processing, such as detecting the location of the LEGO bricks in the image or calculating their orientation."""

def getDepthAxis(height, lego):
    X, Y, Z = (int(x) for x in lego[1:8:3])
    #Z = (0.038, 0.057) X = (0.031, 0.063) Y = (0.031, 0.063, 0.095, 0.127)
    rapZ = height / 0.019 - 1
    pinZ = round(rapZ)
    rapXY = height / 0.032
    pinXY = round(rapXY)
    errZ = abs(pinZ - rapZ) + max(pinZ - 2, 0)
    errXY = abs(pinXY - rapXY) + max(pinXY - 4, 0)
    
    if errZ < errXY:
        return pinZ, 2, pinZ == Z    # pin, is ax Z, match
    else:
        if pinXY == Y: return pinXY, 1, True
        else: return pinXY, 0, pinXY == X
        
        
    """This is a Python function called "getDepthAxis" that takes two inputs: "height", which is the height of the LEGO brick in meters, and "lego", which is a list of seven integers representing the position and orientation of the LEGO brick. The function returns a tuple containing three values: "pin", which is the pin number on the LEGO brick that is closest to the depth axis; "axis", which is an integer representing the orientation of the depth axis relative to the LEGO brick (0 for X-axis, 1 for Y-axis, 2 for Z-axis); and "match", which is a boolean value indicating whether the closest pin on the LEGO brick matches the specified depth axis.

The function calculates the closest pin number to the depth axis by dividing the height of the LEGO brick by the standard length of a LEGO pin (0.019 meters) and subtracting 1. The resulting value is rounded to the nearest integer, since pins are discrete and cannot be placed at non-integer positions. The function then calculates the error between the calculated pin number and the actual pin number for the X, Y, and Z axes. The error for the Z axis is calculated separately from the error for the X and Y axes, since the Z axis can only be aligned with the pins that are oriented vertically.

The function then compares the errors for the X and Y axes with the error for the Z axis to determine which axis is closest to the LEGO brick. If the error for the Z axis is smaller than the errors for the X and Y axes, the function returns the pin number closest to the Z axis and indicates that the depth axis is aligned with the Z axis. Otherwise, the function returns the pin number closest to the X or Y axis, depending on which axis has a smaller error, and indicates whether the closest pin matches the X or Y axis.

In the context of LEGO robotics, this function could be used to determine the orientation of a LEGO brick relative to a depth sensor or camera by analyzing the position of the pins on the brick. This information could then be used to perform tasks such as grasping the LEGO brick with a robotic arm or assembling the LEGO brick with other LEGO pieces."""

def point_distorption(point, height, origin):
    p = dist_tavolo / (dist_tavolo - height)
    point = point - origin
    return p * point + origin
    
"""This is a Python function called "point_distorption" that takes three inputs: "point", which is a tuple of two integers representing the x and y coordinates of a point in an image; "height", which is the height of the LEGO brick in meters; and "origin", which is a tuple of two integers representing the x and y coordinates of the center of the image. The function returns a tuple of two integers representing the distorted coordinates of the input point.

The function first calculates the "p" factor by dividing the distance between the LEGO brick and the camera (dist_tavolo) by the difference between the distance to the LEGO brick and the height of the LEGO brick. This factor is used to scale the distance between the camera and the input point.

The function then subtracts the origin from the input point to center it around the origin. This is because the distortion caused by the LEGO brick affects points relative to the center of the image, rather than absolute positions.

Finally, the function scales the centered point by the "p" factor and adds the origin back to get the distorted position.

In the context of LEGO robotics, this function could be used to correct for the distortion caused by the presence of a LEGO brick in an image taken by a camera or depth sensor. This would enable more accurate localization and tracking of the LEGO brick for robotic manipulation tasks."""

def point_inverse_distortption(point, height):
    p = dist_tavolo / (dist_tavolo - height)
    point = point - origin
    return point / p + origin

def myimshow(title, img):
    def mouseCB(event,x,y,a,b):
        print(x, y, img[y, x], "\r",end='',flush=True)
        print("\033[K", end='')
    cv.imshow(title, img)
    cv.setMouseCallback(title, mouseCB)
    cv.waitKey()
    
    """This is a Python function called "myimshow" that takes two inputs: "title", which is a string representing the title of the image window to be displayed; and "img", which is a NumPy array representing the image data to be displayed.

The function first defines an inner function called "mouseCB" which takes five inputs: "event", "x", "y", "a", and "b". This function prints the x and y coordinates of the mouse click, as well as the value of the pixel at the clicked location. It does this using the "print" function, followed by a "\r" character to move the cursor back to the beginning of the line, and the "flush=True" argument to ensure that the output is immediately displayed in the console. Finally, it uses the "\033[K" escape sequence to clear the current line, ensuring that the output of the next mouse click is displayed cleanly.

The function then displays the input image using the "imshow" function from the OpenCV library. It sets the mouse callback function to be the "mouseCB" function defined earlier, so that the pixel values are printed whenever the user clicks on the image. Finally, it waits for a key press from the user using the "waitKey" function from OpenCV.

In the context of LEGO robotics, this function could be used to display images captured by a camera or depth sensor and allow the user to interactively explore the image data and the underlying pixel values."""

# ----------------- LOCALIZATION ----------------- #


def process_item(imgs, item):

    #images
    rgb, hsv, depth, img_draw = imgs
    #obtaining Yolo informations (class, coordinates, center)
    x1, y1, x2, y2, cn, cl, nm = item.values()
    mar = 15
    x1, y1 = max(mar, x1), max(mar, y1)
    x2, y2 = min(rgb.shape[1]-mar, x2), min(rgb.shape[0]-mar, y2)
    boxMin = np.array((x1-mar, y1-mar))
    x1, y1, x2, y2 = np.int0((x1, y1, x2, y2))

    boxCenter = (y2 + y1) // 2, (x2 + x1) // 2
    color = get_lego_color(boxCenter, rgb)
    hsvcolor = get_lego_color(boxCenter, hsv)
    
    sliceBox = slice(y1-mar, y2+mar), slice(x1-mar, x2+mar)

    #crop img with coordinate bounding box; computing all imgs
    l_rgb = rgb[sliceBox]
    l_hsv = hsv[sliceBox]

    if a_show: cv.rectangle(img_draw, (x1,y1),(x2,y2), color, 2)

    l_depth = depth[sliceBox]

    l_mask = get_lego_mask(hsvcolor, l_hsv) # filter mask by color
    l_mask = np.where(l_depth < dist_tavolo, l_mask, 0)

    l_depth = np.where(l_mask != 0, l_depth, dist_tavolo)


    #myimshow("asda", hsv)
    #getting lego height from camera and table
    l_dist = get_lego_distance(l_depth)
    l_height = dist_tavolo - l_dist
    #masking 
    l_top_mask = cv.inRange(l_depth, l_dist-0.002, l_dist+0.002)
    #cv.bitwise_xor(img_draw,img_draw,img_draw, mask=cv.inRange(depth, l_dist-0.002, l_dist+0.002))
    #myimshow("hmask", l_top_mask)

    # model detect orientation
    depth_borded = np.zeros(depth.shape, dtype=np.float32)
    depth_borded[sliceBox] = l_depth

    depth_image = cv.normalize(
        depth_borded, None, alpha=0, beta=255, norm_type=cv.NORM_MINMAX, dtype=cv.CV_8U
    )
    depth_image = cv.cvtColor(depth_image, cv.COLOR_GRAY2RGB).astype(np.uint8)
   
    #yolo in order to keep te orientation
    #print("Model orientation: Start...", end='\r')
    model_orientation.conf = 0.7
    results = model_orientation(depth_image)
    pandino = []
    pandino = results.pandas().xyxy[0].to_dict(orient="records")
    
    n = len(pandino)
    #print("Model orientation: Finish", n)
    
    
    # Adjust prediction
    pinN, ax, isCorrect = getDepthAxis(l_height, nm)
    if not isCorrect and ax == 2:
        if cl in (1,2,3) and pinN in (1, 2):    # X1-Y2-Z*, *1,2,2-CHAMFER
            cl = 1 if pinN == 1 else 2          # -> Z'pinN'
        elif cl in (7, 8) and pinN in (1, 2):   # X1-Y4-Z*, *1,2
            cl = 7 if pinN == 1 else 8          # -> Z'pinN'
        elif pinN == -1:
            nm = "{} -> {}".format(nm, "Target")
        else:
            print("[Warning] Error in classification")
    elif not isCorrect:
        ax = 1
        if cl in (0, 2, 5, 8) and pinN <= 4:    # X1-Y*-Z2, *1,2,3,4
            cl = (2, 5, 8)[pinN-2]              # -> Y'pinN'
        elif cl in (1, 7) and pinN in (2, 4):   # X1-Y*-Z1, *2,4
            cl = 1 if pinN == 2 else 7          # -> Y'pinN'
        elif cl == 9 and pinN == 1:     # X2-Y2-Z2
            cl = 2                      # -> X1
            ax = 0
        else: print("[Warning] Error in classification")
    nm = legoClasses[cl]

    if n != 1:
        print("[Warning] Classification not found")
        or_cn, or_cl, or_nm = ['?']*3
        or_nm = ('lato', 'lato', 'sopra/sotto')[ax]
    else:
        print()
        #obtaining Yolo informations (class, coordinates, center)
        or_item = pandino[0]
        or_cn, or_cl, or_nm = or_item['confidence'], or_item['class'], or_item['name']
        if or_nm == 'sotto': ax = 2 
        if or_nm == 'lato' and ax == 2: ax = 1
    #---



    #creating silouette top surface lego
    contours, hierarchy = cv.findContours(l_top_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    top_center = np.zeros(2)
    for cnt in contours:
        tmp_center, top_size, top_angle = cv.minAreaRect(cnt)
        top_center += np.array(tmp_center)
    top_center = boxMin + top_center / len(contours)
    #creating silouette lego
    contours, hierarchy = cv.findContours(l_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    if len(contours) == 0: return None
    cnt = contours[0]
    l_center, l_size, l_angle = cv.minAreaRect(cnt)
    l_center += boxMin
    if ax != 2: l_angle = top_angle
    l_box = cv.boxPoints((l_center, l_size, l_angle))
    
    
    if l_size[0] <=3 or l_size[1] <=3:
        cv.drawContours(img_draw, np.int0([l_box]), 0, (0,0,0), 2)
        return None # filter out artifacts
    
    if a_show: cv.drawContours(img_draw, np.int0([l_box]), 0, color, 2)
    

    # silouette distorption
    # get vertexs distance from origin 
    top_box = l_box.copy()
    vertexs_norm = [(i, np.linalg.norm(vec - origin)) for vec, i in zip(l_box, range(4))]
    vertexs_norm.sort(key=lambda tup: tup[1])
    # get closest vertex
    iver = vertexs_norm[0][0]
    vec = l_box[iver]
    # distorping closest vertex
    if or_nm == 'sopra': l_height -= 0.019
    top_box[iver] = point_distorption(l_box[iver], l_height, origin)
    v0 = top_box[iver] - vec
    # adapt adiacents veretx
    v1 = l_box[iver - 3] - vec    # i - 3 = i+1 % 4
    v2 = l_box[iver - 1] - vec

    top_box[iver - 3] += np.dot(v0, v2) / np.dot(v2, v2) * v2
    top_box[iver - 1] += np.dot(v0, v1) / np.dot(v1, v1) * v1

    l_center = (top_box[0] + top_box[2]) / 2

    if a_show:
        cv.drawContours(img_draw, np.int0([top_box]), 0, (5,5,5), 2)
        cv.circle(img_draw, np.int0(top_box[iver]),1, (0,0,255),1,cv.LINE_AA)    



    #rotation and axis drawing
    if or_nm in ('sopra', 'sotto', 'sopra/sotto', '?'): # fin x, y directions (dirX, dirY)
        dirZ = np.array((0,0,1))
        if or_nm == 'sotto': dirZ = np.array((0,0,-1))
        
        projdir = l_center - top_center
        if np.linalg.norm(projdir) < l_size[0] / 10:
            dirY = top_box[0] - top_box[1]
            dirX = top_box[0] - top_box[-1]
            if np.linalg.norm(dirY) < np.linalg.norm(dirX): dirX, dirY = dirY, dirX
            projdir = dirY * np.dot(dirY, projdir)
        edgeFar = [ver for ver in top_box if np.dot(ver - l_center, projdir) >= 0][:2]
        dirY = (edgeFar[0] + edgeFar[1]) / 2 - l_center
        dirY /= np.linalg.norm(dirY)
        dirY = np.array((*dirY, 0))
        dirX = np.cross(dirZ, dirY)

    elif or_nm == "lato": # find pin direction (dirZ)
        edgePin = [ver for ver in top_box if np.dot(ver - l_center, l_center - top_center) >= 0][:2]
        
        dirZ = (edgePin[0] + edgePin[1]) / 2 - l_center
        dirZ /= np.linalg.norm(dirZ)
        dirZ = np.array((*dirZ, 0))
        
        if cl == 10:
            if top_size[1] > top_size[0]: top_size = top_size[::-1]
            if top_size[0] / top_size[1] < 1.7: ax = 0
        if ax == 0:
            vx,vy,x,y = cv.fitLine(cnt, cv.DIST_L2,0,0.01,0.01)
            dir = np.array((vx, vy))
            vertexs_distance = [abs(np.dot(ver - l_center, dir)) for ver in edgePin]
            iverFar = np.array(vertexs_distance).argmin()
            
            dirY = edgePin[iverFar] - edgePin[iverFar-1]
            dirY /= np.linalg.norm(dirY)
            dirY = np.array((*dirY, 0))
            dirX = np.cross(dirZ, dirY)
            if a_show: cv.circle(img_draw, np.int0(edgePin[iverFar]), 5, (70,10,50), 1)
            #cv.line(img_draw, np.int0(l_center), np.int0(l_center+np.array([int(vx*100),int(vy*100)])),(0,0,255), 3)
        if ax == 1:
            dirY = np.array((0,0,1))
            dirX = np.cross(dirZ, dirY)

        if a_show: cv.line(img_draw, *np.int0(edgePin), (255,255,0), 2)

    l_center = point_inverse_distortption(l_center, l_height)

    # post rotation extra
    theta = 0
    if cl == 1 and ax == 1: theta = 1.715224 - np.pi / 2
    if cl == 3 and or_nm == 'sotto': theta = 2.359515 - np.pi
    if cl == 4 and ax == 1: theta = 2.145295 - np.pi
    if cl == 6 and or_nm == 'sotto': theta = 2.645291 - np.pi
    if cl == 10 and or_nm == 'sotto': theta = 2.496793 - np.pi

    rotX = PyQuaternion(axis=dirX, angle=theta)
    dirY = rotX.rotate(dirY)
    dirZ = rotX.rotate(dirZ)

    if a_show:
        # draw frame
        lenFrame = 50
        unit_z = 0.031
        unit_x = 22 * 0.8039 / dist_tavolo
        x_to_z = lenFrame * unit_z/unit_x
        center = np.int0(l_center)

        origin_from_top = origin - l_center
     
        endX = point_distorption(lenFrame * dirX[:2], x_to_z * dirX[2], origin_from_top)
        frameX = (center, center + np.int0(endX))

        endY = point_distorption(lenFrame * dirY[:2], x_to_z * dirY[2], origin_from_top)
        frameY = (center, center + np.int0(endY))
        
        endZ = point_distorption(lenFrame * dirZ[:2], x_to_z * dirZ[2], origin_from_top)
        frameZ = (center, center + np.int0(endZ))
        
        cv.line(img_draw, *frameX, (0,0,255), 2)
        cv.line(img_draw, *frameY, (0,255,0), 2)
        cv.line(img_draw, *frameZ, (255,0,0), 2)
        # ---

        # draw text
        if or_cl != '?': or_cn = ['SIDE', 'UP', 'DOWN'][or_cl]
        text = "{} {:.2f} {}".format(nm, cn, or_cn)
        (text_width, text_height) = cv.getTextSize(text, cv.FONT_HERSHEY_DUPLEX, 0.4, 1)[0]
        text_offset_x = boxCenter[1] - text_width // 2
        text_offset_y = y1 - text_height
        box_coords = ((text_offset_x - 1, text_offset_y + 1), (text_offset_x + text_width + 1, text_offset_y - text_height - 1))
        cv.rectangle(img_draw, box_coords[0], box_coords[1], (210,210,10), cv.FILLED)
        cv.putText(img_draw, text, (text_offset_x, text_offset_y), cv.FONT_HERSHEY_DUPLEX, 0.4, (255, 255, 255), 1)

    def getAngle(vec, ax):
        vec = np.array(vec)
        if not vec.any(): return 0
        vec = vec / np.linalg.norm(vec)
        wise = 1 if vec[-1] >= 0 else -1
        dotclamp = max(-1, min(1, np.dot(vec, np.array(ax))))
        return wise * np.arccos(dotclamp)

    msg = ModelStates()
    msg.name = nm
    #fov = 1.047198
    #rap = np.tan(fov)
    #print("rap: ", rap)
    xyz = np.array((l_center[0], l_center[1], l_height / 2 + height_tavolo))
    xyz[:2] /= rgb.shape[1], rgb.shape[0]
    xyz[:2] -= 0.5
    xyz[:2] *= (-0.968, 0.691)
    xyz[:2] *= dist_tavolo / 0.84
    xyz[:2] += cam_point[:2]

    rdirX, rdirY, rdirZ = dirX, dirY, dirZ
    rdirX[0] *= -1
    rdirY[0] *= -1
    rdirZ[0] *= -1 
    qz1 = PyQuaternion(axis=(0,0,1), angle=-getAngle(dirZ[:2], (1,0)))
    rdirZ = qz1.rotate(dirZ)
    qy2 = PyQuaternion(axis=(0,1,0), angle=-getAngle((rdirZ[2],rdirZ[0]), (1,0)))
    rdirX = qy2.rotate(qz1.rotate(rdirX))
    qz3 = PyQuaternion(axis=(0,0,1), angle=-getAngle(rdirX[:2], (1,0)))

    rot = qz3 * qy2 * qz1
    rot = rot.inverse
    msg.pose = Pose(Point(*xyz), Quaternion(x=rot.x,y=rot.y,z=rot.z,w=rot.w))
    
    #pub.publish(msg)
    #print(msg)
    return msg


#image processing
def process_image(rgb, depth):    
    
    img_draw = rgb.copy()
    hsv = cv.cvtColor(rgb, cv.COLOR_BGR2HSV)

    get_dist_tavolo(depth, hsv, img_draw)
    get_origin(rgb)

    #results collecting localization

    #print("Model localization: Start...",end='\r')
    model.conf = 0.6
    results = model(rgb)
    pandino = results.pandas().xyxy[0].to_dict(orient="records")
    #print("Model localization: Finish  ")
        
    # ----
    if depth is not None:
        imgs = (rgb, hsv, depth, img_draw)
        results = [process_item(imgs, item) for item in pandino]
    
    # ----

    msg = ModelStates()
    for point in results:
        if point is not None:
            msg.name.append(point.name)
            msg.pose.append(point.pose)
    pub.publish(msg)


    if a_show:
        cv.imshow("vision-results.png", img_draw)
        cv.waitKey()

    pass

def process_CB(image_rgb, image_depth):
    t_start = time.time()
    #from standard message image to opencv image
    rgb = CvBridge().imgmsg_to_cv2(image_rgb, "bgr8")                                                
    depth = CvBridge().imgmsg_to_cv2(image_depth, "32FC1")
    
    process_image(rgb, depth)

    print("Time:", time.time() - t_start)
    rospy.signal_shutdown(0)
    pass

#init node function
def start_node():
    global pub

    print("Starting Node Vision 1.0")

    rospy.init_node('vision') 
    
    print("Subscribing to camera images")
    #topics subscription
    rgb = message_filters.Subscriber("/camera/color/image_raw", Image)
    depth = message_filters.Subscriber("/camera/depth/image_raw", Image)
    
    #publisher results
    pub=rospy.Publisher("lego_detections", ModelStates, queue_size=1)

    print("Localization is starting.. ")
    print("(Waiting for images..)", end='\r'), print(end='\033[K')
    
    #images synchronization
    syncro = message_filters.TimeSynchronizer([rgb, depth], 1, reset=True)
    syncro.registerCallback(process_CB)
    
    #keep node always alive
    rospy.spin() 
    pass

def load_models():
    global model, model_orientation
    
    #yolo model and weights classification
    print("Loading model best.pt")
    weight = path.join(path_weigths, 'best.pt')
    model = torch.hub.load(path_yolo,'custom',path=weight, source='local')

    #yolo model and weights orientation
    print("Loading model orientation.pt")
    weight = path.join(path_weigths, 'depth.pt')
    model_orientation = torch.hub.load(path_yolo,'custom',path=weight, source='local')
    pass

if __name__ == '__main__':

    load_models()
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
