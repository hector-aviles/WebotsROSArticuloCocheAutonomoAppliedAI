#!/usr/bin/env python3
"""
Save still images from RGB camera
 
"""
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


#
# Image callback. For detecting lane borders, the following steps are performed:
# - Crop image to consider only the part below horizon
# - Conversion to grayscale, gaussian filtering and Canny border detection
# - Detect lines from borders using Hough transform
# - Filter lines to keep only the ones which are more likely to be a lane border
# - Perform a weighted average
# - Publish detected lanes
#
def callback_rgb_image(msg):
    global writer
    global width, height

    bridge = CvBridge()
    img   = bridge.imgmsg_to_cv2(msg, 'bgr8')
    dsize = (width, height)
    #resize image
    output = cv2.resize(img, dsize)
    #img   = img[int(0.4*img.shape[0]):int(0.97*img.shape[0]) ,:,:]
    writer.write(output)

    cv2.imshow("Scaled image", output)
    cv2.waitKey(10)

def main():
    global writer
    global width, height
    width = 640
    height = 480

    print("Running image capture...")
    rospy.init_node("image_capture")
    rospy.Subscriber('/camera/rgb/raw', Image, callback_rgb_image)

    writer= cv2.VideoWriter('video.mp4', cv2.VideoWriter_fourcc(*'DIVX'), 20, (width, height))
        
    rate = rospy.Rate(10)
    
    rospy.spin()
    

if __name__ == "__main__":
    try:
        main()
    except:
        rospy.ROSInterruptException
        pass

    

