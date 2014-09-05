/* Copyright (C) Keerthi Raj Nagaraja - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Keerthi Raj Nagaraja <nagaraj1@purdue.edu>, July 2014
 */

/* Canny Edge detector */
/* Uses OpenCV's Canny() operator to do edge detection */
/* User has to supply the low threshold(0 to 100) as a command line input while executing the program */
/* Listens to "image_publisher/image" topic for an RGB Color image of any size */
/* Applies canny edge detector after noise reduction by 3x3 simple averaging filter */
/* Publishes the edge image as "/canny_edge_detector/edge_image" topic in 8-bit grayscale image format */
/* Publishes the original image as "/canny_edge_detector/original_image" topic in its original format */
/* Command Line Arguments:	*/
/*  	low threshold for edge detection (Ex: 30) - Range is from 0 to 100 */
/* ------------------------------------------------------------------------------------------------------------ */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointField.h>	
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

#define LOW_THRESHOLD_FOR_CANNY 30
#define MAX_LOW_THRESHOLD 100
#define RATIO_BW_LOW_HIGH_THRESHOLD 3
#define KERNEL_SIZE_FOR_CANNY 3

static const std::string OPENCV_WINDOW = "Detected Edges"; // A name for the OpenCV Image window 

using namespace cv;

/* Global variables */

Mat src_gray;
Mat detected_edges;
int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = MAX_LOW_THRESHOLD;
int ratio = RATIO_BW_LOW_HIGH_THRESHOLD;
int kernel_size = KERNEL_SIZE_FOR_CANNY;

/* ------------------------------------------------------------------------------------------------------------ */
/* Noise reduction, Canny Edge detection and Displaying the result */
static void CannyThreshold(int, void*)
{
    blur( src_gray, detected_edges, Size(3,3) ); 					   // Reduce noise with a 3x3 kernel
    Canny(detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size ); // Apply Canny detector
    imshow(OPENCV_WINDOW, detected_edges );						   // we display our result
    cv::waitKey(1);
}
/* ------------------------------------------------------------------------------------------------------------ */

/* ------------------------------------------------------------------------------------------------------------ */
/* Class containing ROS nodehadler, Image transporter and functions to subscribe, process and publish the images */
class CannyEdgeDetector
{
  ros::NodeHandle nh_;				 // ROS node handler
  image_transport::ImageTransport it_;		 // Image transporter 
  image_transport::Subscriber image_sub_;	 // Image subscriber of original image
  image_transport::Publisher image_pub_;	 // Image publisher of processed image
  image_transport::Publisher image_pub_original; // Image publisher of original image
  public:
  CannyEdgeDetector()				 // Constructor	
    : it_(nh_)
  {
    /* Subscribe to input video feed and publish output video feed along with original video */
    image_sub_ = it_.subscribe("image_publisher/image", 1, &CannyEdgeDetector::imageCb, this);
    image_pub_ = it_.advertise("/canny_edge_detector/edge_image", 1);
    image_pub_original = it_.advertise("/canny_edge_detector/original_image", 1);
    cv::namedWindow(OPENCV_WINDOW, WINDOW_NORMAL);
  }

  ~CannyEdgeDetector()				// Destructor
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  /* Function to be executed for each incoming image frame*/
  void imageCb(const sensor_msgs::ImageConstPtr& msg)	
  {
    cv_bridge::CvImagePtr cv_ptr;	// Declare a pointer to opencv image
    
    try		// Try converting the ROS image to OpenCV image format
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)	// Catch exception if it occurs
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cvtColor( cv_ptr->image, src_gray, COLOR_BGR2GRAY );	// Convert the image from BGR to grayscale

    CannyThreshold(0, 0);					// Apply Canny Edge Detector
    
    /* Fill in the header, encoding and image containers of OpenCV image format */
    cv_ptr->header   = msg->header; 			    // Same timestamp and tf frame as input image
    cv_ptr->encoding = sensor_msgs::image_encodings::MONO8; // Image encoding
    cv_ptr->image    = detected_edges;  		    // Your cv::Mat
    
    image_pub_.publish(cv_ptr->toImageMsg());	// Convert the OpenCV image to ROS image format and publish it
    image_pub_original.publish(msg);		// Publish the original incoming image too
  }  
};
/* ------------------------------------------------------------------------------------------------------------ */


/* ------------------------------------------------------------------------------------------------------------ */
/* main function */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "canny_edge_detector");
  lowThreshold = atoi(argv[1]);			// Get the edge detection low threshold from user as a command line argument
  CannyEdgeDetector ic;				// declare an object for class "CannyEdgeDetector"
  ros::spin();					// Let the ROS core take over the execution based on incoming messages
  return 0;
} 
/* ------------------------------------------------------------------------------------------------------------ */
