import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import numpy as np
import pyrealsense2 as rs2
from std_msgs.msg import String, Float64
import matplotlib.pyplot as plt 
if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2
distanceDepth = 0.0
cloestpoint = rospy.Publisher("/NearestPoint", Float64, queue_size=1)
count  = 0
x_coordinates = []
y_coordinates = []
breathing_count = 0
initial_time = 0
val =  False

# Teleop only
breathing_only = "breathing" in sys.argv


class ImageListener:
    def __init__(self, depth_image_topic, depth_info_topic):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(depth_image_topic, msg_Image, self.imageDepthCallback)
        self.sub_info = rospy.Subscriber(depth_info_topic, CameraInfo, self.imageDepthInfoCallback)
        confidence_topic = depth_image_topic.replace('depth', 'confidence')
        self.sub_conf = rospy.Subscriber(confidence_topic, msg_Image, self.confidenceCallback)
        self.intrinsics = None
        self.pix = None
        self.pix_grade = None

    def imageDepthCallback(self, data):
        global distanceDepth

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            # pick one pixel among all the pixels with the closest range:
            indices = np.array(np.where(cv_image == cv_image[cv_image > 0].min()))[:,0]
            pix = (indices[1], indices[0])
            self.pix = pix
            line = '\rDepth at pixel(%3d, %3d): %7.1f(mm).' % (pix[0], pix[1], cv_image[pix[1], pix[0]])
            distanceDepth = cv_image[pix[1], pix[0]] # ensure that the distance is correct, we can spit out a boolean or a value I will opt for a value in meters
            # print("AYE")
            # print(distanceDepth)

            if self.intrinsics:
                depth = cv_image[pix[1], pix[0]]
                result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
                line += '  Coordinate: %8.2f %8.2f %8.2f.' % (result[0], result[1], result[2])
            if (not self.pix_grade is None):
                line += ' Grade: %2d' % self.pix_grade
            line += '\r'
            sys.stdout.write(line)
            sys.stdout.flush()
            cloestpoint.publish(Float64(distanceDepth))

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return

    def confidenceCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            grades = np.bitwise_and(cv_image >> 4, 0x0f)
            if (self.pix):
                self.pix_grade = grades[self.pix[1], self.pix[0]]
        except CvBridgeError as e:
            print(e)
            return



    def imageDepthInfoCallback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.D]
        except CvBridgeError as e:
            print(e)
            return

def breathing():
    
    global breathing_count
    global y_coordinates
    global x_coordinates
    global initial_time
    global distanceDepth
    global count


    x_coordinates.append(count) # get the time in integer we can also do float: rospy.get_time()
    #y_coordinates.append([1,3,4,8]) #Float64(distanceDepth)
    #x_coordinates.append([0,1,2,4])
    y_coordinates.append(float(distanceDepth))
    count = count + 1
    print(count)
    
    if(len(x_coordinates) == 70):
        
        print(x_coordinates[:10], y_coordinates[:10])
        print(len(x_coordinates))
        plt.plot(x_coordinates, y_coordinates)

        plt.xlabel('Time Step (S)')
        plt.ylabel('Chest Height')
        plt.title('Patient 0212 Breathing')
        plt.savefig("Breathing.png")
        plt.show()
        val = False
        #rospy.sleep(50)

def main():

    depth_image_topic = 'cam_mr/depth/image_rect_raw'
    depth_info_topic = 'cam_mr/depth/camera_info'
    # closestpoint is the name of the rospy publisher
    global breathing_only
    global val
    print ('')
    print ('show_center_depth.py')
    print ('--------------------')
    print ('App to demontrate the usage of the /camera/depth topics.')
    print ('')
    print ('Application subscribes to %s and %s topics.' % (depth_image_topic, depth_info_topic))
    print ('Application then calculates and print the range to the closest object.')
    print ('If intrinsics data is available, it also prints the 3D location of the object')
    print ('If a confedence map is also available in the topic %s, it also prints the confidence grade.' % depth_image_topic.replace('depth', 'confidence'))
    print ('')
    
    listener = ImageListener(depth_image_topic, depth_info_topic)  
    
    r = rospy.Rate(10)
    if breathing_only:

        while not rospy.core.is_shutdown():
            breathing()   
            r.sleep()    
            # rospy.spin()
            print("ok") 
    else:
        rospy.spin()

if __name__ == '__main__':
    node_name = os.path.basename(sys.argv[0]).split('.')[0]
    rospy.init_node(node_name)
    main()
