#!/usr/bin/env python
"""Turtlebot Follower Node."""
import math
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
import mediapipe as mp
import matplotlib.pyplot as plt
from cv_bridge import CvBridge 
import numpy as np
from typing import Tuple

class Follower():
    def __init__(
        self, 
        rate: int = 50,
        speed: float = 1,
        spin_speed: float = 2,
    ):
        """Initialize the following algorithm params."""
        self.rate = rate
        self.speed = speed 
        self.spin_speed = spin_speed

        # Robot states
        self.moving = False
        self.com = Twist() # current command
        self.mp_pose = mp.solutions.pose 
        # Setting up the Pose function.
        self.pose = self.mp_pose.Pose(static_image_mode=True, min_detection_confidence=0.3, model_complexity=2)
        # Initializing mediapipe drawing class, useful for annotation.
        self.mp_drawing = mp.solutions.drawing_utils 
        # Setup Pose function for video.
        self.pose_video = self.mp_pose.Pose(static_image_mode=False, min_detection_confidence=0.5, model_complexity=1)


    def image_callback(self, msg: Image):
        """Process the image data."""
        # Get Image from Camera
        br = CvBridge()
        rospy.loginfo("receiving video frame")
        current_frame = br.imgmsg_to_cv2(msg)
        cv_image_array = np.array(current_frame, dtype = np.dtype('f8'))
        cv_image_norm = cv2.normalize(cv_image_array, None, 0, 1, cv2.NORM_MINMAX)
        print(cv_image_norm.shape)
        
        # Run Pose Detection
        # frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
        frame = current_frame

        # Flip the frame horizontally for natural (selfie-view) visualization.
        frame = cv2.flip(frame, 1)
        
        # Get the width and height of the frame
        frame_height, frame_width, _ =  frame.shape
        
        # Resize the frame while keeping the aspect ratio.
        frame = cv2.resize(frame, (int(frame_width * (640 / frame_height)), 640))
        
        # Perform Pose landmark detection.
        frame, landmarks = self.detectPose(frame, self.pose_video, display=False)
        
        # Perform the pose classification.
        if landmarks:
            self.classifyPose(landmarks, frame, display=False)

        # Display the frame.
        cv2.imshow('Pose Detection', frame)
        cv2.waitKey(1)


    def detectPose(self, image, pose, display=True):
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
                self.mp_drawing.draw_landmarks(image=output_image, landmark_list=results.pose_landmarks,
                                        connections=self.mp_pose.POSE_CONNECTIONS)
                
                # Iterate over the detected landmarks.
                for landmark in results.pose_landmarks.landmark:
                    
                    # Append the landmark into the list.
                    landmarks.append((int(landmark.x * width), int(landmark.y * height),
                                        (landmark.z * width)))
            
            if display:
            
                # Display the original input image and the resultant image.
                plt.figure(figsize=[22,22])
                plt.subplot(121);plt.imshow(image[:,:,::-1]);plt.title("Original Image");plt.axis('off');
                plt.subplot(122);plt.imshow(output_image[:,:,::-1]);plt.title("Output Image");plt.axis('off');
                
                # Also Plot the Pose landmarks in 3D.
                self.mp_drawing.plot_landmarks(results.pose_world_landmarks, self.mp_pose.POSE_CONNECTIONS)
                
            else:
                # Return the output image and the found landmarks.
                return output_image, landmarks
            

    def calculateAngle(self, landmark1, landmark2, landmark3) -> float:
        '''
        This function calculates angle between three different landmarks.
        Args:
            landmark1: The first landmark containing the x,y and z coordinates.
            landmark2: The second landmark containing the x,y and z coordinates.
            landmark3: The third landmark containing the x,y and z coordinates.
        Returns:
            angle: The calculated angle between the three landmarks.

        '''

        # Get the required landmarks coordinates.
        x1, y1, _ = landmark1
        x2, y2, _ = landmark2
        x3, y3, _ = landmark3

        # Calculate the angle between the three points
        angle = math.degrees(math.atan2(y3 - y2, x3 - x2) - math.atan2(y1 - y2, x1 - x2))
        
        # Check if the angle is less than zero.
        if angle < 0:

            # Add 360 to the found angle.
            angle += 360
        
        # Return the calculated angle.
        return angle
    

    def classifyPose(self, landmarks, output_image, display=False) -> Tuple[np.ndarray, str]:
        '''
        This function classifies yoga poses depending upon the angles of various body joints.
        Args:
            landmarks: A list of detected landmarks of the person whose pose needs to be classified.
            output_image: A image of the person with the detected pose landmarks drawn.
            display: A boolean value that is if set to true the function displays the resultant image with the pose label 
            written on it and returns nothing.
        Returns:
            output_image: The image with the detected pose landmarks drawn and pose label written.
            label: The classified pose label of the person in the output_image.

        '''
        
        # Initialize the label of the pose. It is not known at this stage.
        label = 'Unknown Pose'

        # Specify the color (Red) with which the label will be written on the image.
        color = (0, 0, 255)
        
        # Calculate the required angles.
        #----------------------------------------------------------------------------------------------------------------
        
        # Get the angle between the left shoulder, elbow and wrist points. 
        left_elbow_angle = self.calculateAngle(landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER.value],
                                        landmarks[self.mp_pose.PoseLandmark.LEFT_ELBOW.value],
                                        landmarks[self.mp_pose.PoseLandmark.LEFT_WRIST.value])
        
        # Get the angle between the right shoulder, elbow and wrist points. 
        right_elbow_angle = self.calculateAngle(landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER.value],
                                        landmarks[self.mp_pose.PoseLandmark.RIGHT_ELBOW.value],
                                        landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST.value])   
        
        # Get the angle between the left elbow, shoulder and hip points. 
        left_shoulder_angle = self.calculateAngle(landmarks[self.mp_pose.PoseLandmark.LEFT_ELBOW.value],
                                            landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER.value],
                                            landmarks[self.mp_pose.PoseLandmark.LEFT_HIP.value])

        # Get the angle between the right hip, shoulder and elbow points. 
        right_shoulder_angle = self.calculateAngle(landmarks[self.mp_pose.PoseLandmark.RIGHT_HIP.value],
                                            landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER.value],
                                            landmarks[self.mp_pose.PoseLandmark.RIGHT_ELBOW.value])

        # Get the angle between the left hip, knee and ankle points. 
        left_knee_angle = self.calculateAngle(landmarks[self.mp_pose.PoseLandmark.LEFT_HIP.value],
                                        landmarks[self.mp_pose.PoseLandmark.LEFT_KNEE.value],
                                        landmarks[self.mp_pose.PoseLandmark.LEFT_ANKLE.value])

        # Get the angle between the right hip, knee and ankle points 
        right_knee_angle = self.calculateAngle(landmarks[self.mp_pose.PoseLandmark.RIGHT_HIP.value],
                                        landmarks[self.mp_pose.PoseLandmark.RIGHT_KNEE.value],
                                        landmarks[self.mp_pose.PoseLandmark.RIGHT_ANKLE.value])
        
                
        # Check if the both arms are straight.
        if left_elbow_angle > 165 and left_elbow_angle < 195 and right_elbow_angle > 165 and right_elbow_angle < 195:

            # Check if shoulders are at the required angle.
            if left_shoulder_angle > 80 and left_shoulder_angle < 110 and right_shoulder_angle > 80 and right_shoulder_angle < 110:

                # Check if it is the T pose.
                # Check if both legs are straight
                if left_knee_angle > 160 and left_knee_angle < 195 and right_knee_angle > 160 and right_knee_angle < 195:

                    # Specify the label of the pose that is tree pose.
                    label = 'T Pose'


        # Check if the pose is classified successfully
        if label != 'Unknown Pose':
            # Update the color (to green) with which the label will be written on the image.
            color = (0, 255, 0)  
        
        # Write the label on the output image. 
        cv2.putText(output_image, label, (10, 30),cv2.FONT_HERSHEY_PLAIN, 2, color, 2)
        
        if display:
            # Display the resultant image.
            plt.figure(figsize=[10,10])
            plt.imshow(output_image[:,:,::-1]);plt.title("Output Image");plt.axis('off');
            
        else:
            # Return the output image and the classified label.
            return output_image, label
    

    def moveForward(self):
        """Move the robot forward if it should be moving."""
        if self.moving:
            self.com.linear.x = self.speed
            self.com.angular.z = 0


    def main_control_loop(self):
        """Run the follower routine."""
        print("Running follower algorithm...")
        rospy.init_node("follower", anonymous=True)

        # Publisher object that decides what kind of topic to publish and how fast.
        cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        # Subscribe to the Camera to get the image data
        # The callback will run the pose detection algorithm
        rospy.Subscriber("/cv_camera/image_raw", Image, self.image_callback)

        # The main loop will run at a rate in Hz
        rate = rospy.Rate(self.rate)
        
        while not rospy.is_shutdown():
            # Here's where we publish the current commands.
            cmd_vel_pub.publish(self.com)
            # Sleep for as long as needed to achieve the loop rate.
            rate.sleep()


if __name__ == "__main__":
    Follower().main_control_loop()