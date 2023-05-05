#!/usr/bin/env python
import math
# from time import time
import mediapipe as mp
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2 
import numpy as np

def callback(data):
    br = CvBridge()
    rospy.loginfo("receiving video frame")
    current_frame = br.imgmsg_to_cv2(data)
    cv_image_array = np.array(current_frame, dtype = np.dtype('f8'))
    cv_image_norm = cv2.normalize(cv_image_array, None, 0, 1, cv2.NORM_MINMAX)
    # image = cv2.imshow("camera", cv2.cvtColor(current_frame,cv2.COLOR_BGR2RGB))
    image = current_frame
    # detectPose(image, pose, display=True)
    print(cv_image_norm.shape)
    # cv2.waitKey(1)
    
    frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)

    # Flip the frame horizontally for natural (selfie-view) visualization.
    frame = cv2.flip(frame, 1)
    
    # Get the width and height of the frame
    frame_height, frame_width, _ =  frame.shape
    
    # Resize the frame while keeping the aspect ratio.
    frame = cv2.resize(frame, (int(frame_width * (640 / frame_height)), 640))
    
    # Perform Pose landmark detection.
    frame, _ = detectPose(frame, pose_video, display=False)
    
    # Display the frame.
    cv2.imshow('Pose Detection', frame)
    cv2.waitKey(1)
    
def receive_message():
    rospy.init_node('video_sub_py', anonymous=True)
    rospy.Subscriber('/cv_camera/image_raw', Image, callback) # check name by rostopic list
    rospy.spin()
    cv2.destroyAllWindows()

def detectPose(image, pose, display=True):
    '''
    This function performs pose detection on an image.
    Args:
        image: The input image with a prominent person whose pose landmarks needs to be detected.
        pose: The pose setup function required to perform the pose detection.
        display: A boolean value that is if set to true the function displays the original input image, the resultant image, 
                 and the pose landmarks in 3D plot and returns nothing.
    Returns:
        output_image: The input image with the detected pose landmarks drawn.
        landmarks: A list of detected landmarks converted into their original scale.
    '''
    
    # Create a copy of the input image.
    if image is not None:
        output_image = image.copy()
        # output_image = image
        
        # Convert the image from BGR into RGB format.
        imageRGB = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Perform the Pose Detection.
        results = pose.process(imageRGB)
        
        # Retrieve the height and width of the input image.
        height, width, _ = image.shape
        
        # Initialize a list to store the detected landmarks.
        landmarks = []
        
        # Check if any landmarks are detected.
        if results.pose_landmarks:
        
            # Draw Pose landmarks on the output image.
            mp_drawing.draw_landmarks(image=output_image, landmark_list=results.pose_landmarks,
                                      connections=mp_pose.POSE_CONNECTIONS)
            
            # Iterate over the detected landmarks.
            for landmark in results.pose_landmarks.landmark:
                
                # Append the landmark into the list.
                landmarks.append((int(landmark.x * width), int(landmark.y * height),
                                      (landmark.z * width)))
        
        # Check if the original input image and the resultant image are specified to be displayed.
        if display:
        
            # Display the original input image and the resultant image.
            plt.figure(figsize=[22,22])
            plt.subplot(121);plt.imshow(image[:,:,::-1]);plt.title("Original Image");plt.axis('off');
            plt.subplot(122);plt.imshow(output_image[:,:,::-1]);plt.title("Output Image");plt.axis('off');
            
            # Also Plot the Pose landmarks in 3D.
            mp_drawing.plot_landmarks(results.pose_world_landmarks, mp_pose.POSE_CONNECTIONS)
            
        # Otherwise
        else:
            
            # Return the output image and the found landmarks.
            return output_image, landmarks

if __name__ == '__main__':
    # Initializing mediapipe pose class.
    mp_pose = mp.solutions.pose
    # Setting up the Pose function.
    pose = mp_pose.Pose(static_image_mode=True, min_detection_confidence=0.3, model_complexity=2)
    # Initializing mediapipe drawing class, useful for annotation.
    mp_drawing = mp.solutions.drawing_utils 
    # Setup Pose function for video.
    pose_video = mp_pose.Pose(static_image_mode=False, min_detection_confidence=0.5, model_complexity=1)
    # Create named window for resizing purposes
    # cv2.namedWindow('Pose Detection', cv2.WINDOW_NORMAL)
    # Initialize a variable to store the time of the previous frame.
    time1 = 0
    receive_message()


