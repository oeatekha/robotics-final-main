#!/usr/bin/python


import rospy
import numpy as np
from numpy.linalg import inv
import cv2  # OpenCV module

from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion
from std_msgs.msg import ColorRGBA, Float32, Bool
from user_input.msg import Voxel
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import math


# Bridge to convert ROS Image type to OpenCV Image type
cv_bridge = CvBridge()  

class DepthAnalysis(object):
    def __init__(self):
        rospy.init_node('image_processing', anonymous=True)
        rospy.Subscriber("/camera/depth/image_rect_raw/", Image, self.depth_cb)
        self.rectangle_pub = rospy.Publisher("/images/voxels", Voxel, queue_size = 1)
        self.voxel_msg = Voxel()
        r = rospy.Rate(100)

        while not rospy.is_shutdown():
            r.sleep()

    def voxelize(self, depth):
        stop_ind = 0
        eps = 0.0
        #list of lists composed of [length, depth]
        rect_list = []
        while stop_ind != (len(depth)-1):
            rect_length = 0
            min_depth = depth[stop_ind]
            for j in range(stop_ind,len(depth)):
                depth_diff = depth[stop_ind] - depth[j]
                if(np.abs(depth_diff) > eps) or (j == len(depth)-1):
                    rect_list.append([min_depth, rect_length])
                    stop_ind = j
                    break
                else:
                    rect_length = rect_length + 1
                    min_depth = np.amin([min_depth,depth[j]])
        return rect_list

    def depth_cb(self, msg):
        eps = 0
        mindepth_list = self.get_compressed_depth(msg)
        for i in range(0,len(mindepth_list)):
            for j in range(i,len(mindepth_list)):
                if np.abs(mindepth_list[i] - mindepth_list[j]) < eps:
                    #change depths closest distance
                    min_depth = np.amin([mindepth_list[i],mindepth_list[j]])
                    mindepth_list[j] = min_depth
                    mindepth_list[i] = min_depth
        rect_list = self.voxelize(mindepth_list)
        depths = []
        lengths = []
        for i in range(len(rect_list)):
            depths.append(rect_list[i][0])
            lengths.append(rect_list[i][1])
        self.voxel_msg.depths = depths
        self.voxel_msg.lengths = lengths
        self.rectangle_pub.publish(self.voxel_msg)


    def get_compressed_depth(self,data):
        depth_arr = np.array(cv_bridge.imgmsg_to_cv2(data, "16UC1"), dtype = np.float32)
        depth_shape = depth_arr.shape
        compressed_depth = []
        for i in range(0,depth_shape[0]):
            compressed_depth.append(np.amax(depth_arr[i]))
        return np.array(compressed_depth)



# # Task 3 callback
# def rosRGBDCallBack(rgb_data, depth_data):
#     try:
#         cv_image = cv_bridge.imgmsg_to_cv2(rgb_data, "bgr8")
#         cv_depthimage = cv_bridge.imgmsg_to_cv2(depth_data, "32FC1")
#         cv_depthimage2 = np.array(cv_depthimage, dtype=np.float32)
#     except CvBridgeError as e:
#         print(e)

#     contours, mask_image = HSVObjectDetection(cv_image, toPrint = False)

#     for cnt in contours:
#         xp,yp,w,h = cv2.boundingRect(cnt)
        
#         # Get depth value from depth image, need to make sure the value is in the normal range 0.1-10 meter
#         if not math.isnan(cv_depthimage2[int(yp)][int(xp)]) and cv_depthimage2[int(yp)][int(xp)] > 0.1 and cv_depthimage2[int(yp)][int(xp)] < 10.0:
#             zc = cv_depthimage2[int(yp)][int(xp)]
#             print 'zc', zc
#         else:
#             continue
            
#         centerx, centery = xp+w/2, yp+h/2
#         cv2.rectangle(cv_image,(xp,yp),(xp+w,yp+h),[0,255,255],2)
        
#         showPyramid(centerx, centery, zc, w, h)

# def getXYZ(xp, yp, zc, fx,fy,cx,cy):
    
#     xp = np.array(([xp],[yp],[1]))
#     #
#     # Homogenous Transform matrix
#     K = np.matrix([(fx, 0, cx),(0, fy, cy),(0, 0, 1)])
#     # 
#     # Hint: inv() returns the inverse of a matrix
#     xd = inv(K)*xp
#     xn = xd
#     xc = np.array([xd[0]*zc, xd[1]*zc, zc])
#     return (xc[0],xc[1],xc[2])


# def showImage(cv_image, mask_erode_image, mask_image):
#     # Bitwise-AND mask and original image
#     res = cv2.bitwise_and(cv_image, cv_image, mask = mask_image)
    
#     # Draw a cross at the center of the image
#     cv2.line(cv_image, (320, 235), (320, 245), (255,0,0))
#     cv2.line(cv_image, (325, 240), (315, 240), (255,0,0))
    
#     # Show the images in cv window (may freeze in ROS Kinetic/16.04)
#     cv2.imshow('OpenCV_Original', cv_image)
#     #  cv2.imshow('OpenCV_Mask_Erode', mask_erode_image)
#     #  cv2.imshow('OpenCV_Mask_Dilate', mask_image)
#     cv2.imshow('OpenCV_View', res)
#     cv2.waitKey(3)
    
#     # Publish the images to ROS and show it in rviz
    
#     #TO DO: IF YOU WANT TO. You can uncomment these images but it may cause RVIZ to crash.
#     #img_pub1.publish(cv_bridge.cv2_to_imgmsg(cv_image, encoding="passthrough"))
#     #img_pub2.publish(cv_bridge.cv2_to_imgmsg(mask_erode_image, encoding="passthrough"))
#     #img_pub3.publish(cv_bridge.cv2_to_imgmsg(mask_image, encoding="passthrough"))
#     img_pub4.publish(cv_bridge.cv2_to_imgmsg(res, encoding="passthrough"))

# # Create a pyramid using 4 triangles
# def showPyramid(xp, yp, zc, w, h):
#     # X1-X4 are the 4 corner points of the base of the pyramid
#     X1 = getXYZ(xp-w/2, yp-h/2, zc, fx, fy, cx, cy)
#     X2 = getXYZ(xp-w/2, yp+h/2, zc, fx, fy, cx, cy)
#     X3 = getXYZ(xp+w/2, yp+h/2, zc, fx, fy, cx, cy)
#     X4 = getXYZ(xp+w/2, yp-h/2, zc, fx, fy, cx, cy)
#     vis_pub.publish(createTriangleListMarker(1, [X1, X2, X3, X4], rgba = [1,0,0,1], frame_id = '/camera'))

# # Create a list of Triangle markers for visualization
# def createTriangleListMarker(marker_id, points, rgba, frame_id = '/camera'):
#     marker = Marker()
#     marker.header.frame_id = frame_id
#     marker.type = marker.TRIANGLE_LIST
#     marker.scale = Vector3(1,1,1)
#     marker.id = marker_id
    
#     n = len(points)
    
#     if rgba is not None:
#         marker.color = ColorRGBA(*rgba)
        
#     o = Point(0,0,0)
#     for i in xrange(n):
#         p = Point(*points[i])
#         marker.points.append(p)
#         p = Point(*points[(i+1)%4])
#         marker.points.append(p)
#         marker.points.append(o)
        
#     marker.pose = poselist2pose([0,0,0,0,0,0,1])
#     return marker

# def poselist2pose(poselist):
#     return Pose(Point(*poselist[0:3]), Quaternion(*poselist[3:7]))

if __name__=='__main__':
    node = DepthAnalysis()
    
