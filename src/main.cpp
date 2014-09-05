/* Copyright (C) Keerthi Raj Nagaraja - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Keerthi Raj Nagaraja <nagaraj1@purdue.edu>, July 2014
 */
 
 
/* Multiple lane detector - main source file */
/* Uses OpenCV's HoughLinesP() function to extract straight edges in the image */
/* Listens to "/canny_edge_detector/edge_image" topic for an edge detected image of any size */
/* Applies probabilistic hough transform after masking unwanted region of the image to get the straight edges */
/* Publishes the detected straight edges as "/lane_detector/hough_image" ROS topic in BGR8 image format */
/* The extracted edges are clustered using a clustering algorithm and a straight line is fitted for each cluster */
/* The fitted lines are used to generate smooth bezier curves which represents multiple lanes */
/* Publishes the lane detected image as "/lane_detector/lane_detected_image" ROS topic in BGR8 image format */
/* Command Line Arguments:	*/
/*	folder (along with start of the filename) to save the frames(in jpg) with detected lanes - (Ex: /home/raj/data_road/out_fram/out_)*/
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
#include <time.h>
#include <string>
#include <iostream>
#include <algorithm>
#include <lane_detection/parameters.h>			// Contains all algorithm parameters which are configurable
#include <lane_detection/cluster_edges.hpp>
#include <lane_detection/bezier_curve.hpp>

using namespace cv;
using namespace std;

/* ------------------------------------------------------------------------------------------------------------ */
/* Global variables */
int p1_x, p1_y, p2_x, p2_y, p3_x, p3_y; // Points which define the masked portion of the image. Takes values based on MASK parameters in 						   parameters.h
 
Mat cdst, src_gray;		// Mat variables to store the input/output and intermediate images
struct fitted_lines fitted_lines;	// Structure to store output lines from clustering stage
struct bezier_curves curves;		// Structure to store bezier curves generated at the final stage

char* folder_to_save;			// Variable to store directory to save the output frames
	
/* Names for windows which displays images */	
static const std::string DETECTED_LINES = "Detected Lines";  
static const std::string DETECTED_LANES_ON_ORIGINAL_IMAGE = "Detected Lanes On Original Image";
/* ------------------------------------------------------------------------------------------------------------ */



/* ------------------------------------------------------------------------------------------------------------ */
/* Almost all stages of lane detection (subscribtion to edge-image, hough transform, clustering edges, bezier curve generation and publishing lane-detected image) happens within this class */
class LaneDetection					
{
  ros::NodeHandle node;			 // A Ros node		
  image_transport::ImageTransport it;	 // Image transport package provides easy way to subscribe and publish images
  image_transport::Subscriber hough_sub; // Image subscriber which listens to original image
  image_transport::Publisher hough_pub;  // Image publisher which publishes image with straight edges detected by hough transform
  image_transport::Subscriber final_sub; // Image subscriber which listens to original image
  image_transport::Publisher final_pub;	 // Image publisher which publishes lane-detected image
  
public:
  LaneDetection()			// Constructor which initializes subscriptions and publications
    : it(node)
  {
    hough_sub = it.subscribe("/canny_edge_detector/edge_image", 1, &LaneDetection::process, this);
    hough_pub = it.advertise("/lane_detector/hough_image", 1);
    final_sub = it.subscribe("/canny_edge_detector/original_image", 1, &LaneDetection::Publish, this);
    final_pub = it.advertise("/lane_detector/lane_detected_image", 1);  
    cv::namedWindow(DETECTED_LINES, WINDOW_NORMAL);
    cv::namedWindow(DETECTED_LANES_ON_ORIGINAL_IMAGE, WINDOW_NORMAL);
  }

  ~LaneDetection()			// Desctructor which closes the OpenCV window 
  {
    cv::destroyWindow(DETECTED_LINES);
    cv::destroyWindow(DETECTED_LANES_ON_ORIGINAL_IMAGE);
  }
 
  /* Function to be executed after every edge-image frame is received as a sensor message. */
  /* This function receives the edge-image, applies hough transform, does clustering, fits straight lines and then generates bezier curves*/
  void process(const sensor_msgs::ImageConstPtr& msg)	
  {
    cv_bridge::CvImagePtr cv_ptr;		// Pointer to an OpenCV image container
    struct detected_lines detected_lines;	// Structure to store detected lines(edges) from hough transform
    std::vector<struct clustered_lines> groups;	// Lines(edges) clustered into groups. A vector of struct variables. Each struct holds a group of lines	
    				
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);	// Convert OpenCV image to ROS image
    }
    catch (cv_bridge::Exception& e)			// If not, catch an exception
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    cvtColor( cv_ptr->image, src_gray, COLOR_BGR2GRAY );		// Convert image to grayscale
    
    p1_x=0;						// Calculate points which define the masked area of the image
    p1_y=MASK_REGION_CONST_2*src_gray.rows;		// The mask constants are defined in "parameters.h"
    p2_x=MASK_REGION_CONST_3*src_gray.cols;
    p2_y=MASK_REGION_CONST_1*src_gray.rows;
    p3_x=src_gray.cols-1;
    p3_y=MASK_REGION_CONST_2*src_gray.rows;
    
    /* Mask the portion of the image by assigning 0 intensity value to those pixels */
    for(int i=0; i<src_gray.cols; i++)		
    {
    	for(int j=0;j<src_gray.rows;j++)
    	{
    		if(j<p2_y)
    		{
    			src_gray.at<uchar>(j, i) = 0;
    		}
    		else
    		{
    			if(i < (p1_x+(j-p1_y)*(p1_x-p2_x)/(p1_y-p2_y)))
    			{
    				src_gray.at<uchar>(j, i) = 0;
    			}
    			if(i > (p3_x+(j-p3_y)*(p3_x-p2_x)/(p3_y-p2_y)))
    			{
    				src_gray.at<uchar>(j, i) = 0;
    			}
    		}
    		#ifdef CALTECH_DATASET				// Only for Caltech dataset, mask more because of different image size
    		if(j>MASK_REGION_CONST_4*src_gray.rows)	
    		{
    			src_gray.at<uchar>(j, i) = 0;	
    		}
    		#endif
    	}
    }		
    cvtColor(src_gray, cdst, CV_GRAY2BGR );	// Copy the masked grayscale image to an image container and convert it to BGR format
    						// BGR format is only to allow colored lines to be drawn on these images in the final output
    
    vector<Vec4i> lines;			// Vector of pair of points which define lines
    cv::HoughLinesP(src_gray, lines, 1, CV_PI/180, HT_VOTES_THRESHOLD, HT_MIN_LINE_LENGTH, HT_MAX_DIST_BW_LINES);	// Run hough transform on the grayscale image to get the initial lines
    
    /* Extract the lines which have slopes of greater than 10 degrees(absolute) and less than 85 degrees(absolute). Lines with slopes greater than 85 degrees are only allowed if they lie in the central portion of the image */
    int j = 0;
    detected_lines.lines.clear();
    detected_lines.angles.clear();
    detected_lines.intercepts.clear();
    for( size_t i = 0; i < lines.size(); i++ )
    {
       Vec4i l = lines[i];
       int swap;
       if(l[1]<l[3])
       {
       		swap = l[0];
       		l[0]=l[2];
       		l[2]=swap;
       		swap=l[1];
       		l[1]=l[3];
       		l[3]=swap;
       		
       }
       if( atan(((float)abs(l[1]-l[3])/(float)abs(l[0]-l[2])))*180/CV_PI < LOWER_ANGLE_LIMIT)
       {
       	continue;
       }		
       if( atan(((float)abs(l[1]-l[3])/(float)abs(l[0]-l[2])))*180/CV_PI > UPPER_ANGLE_LIMIT)
       {
       		if((l[0] < (src_gray.cols/2)-(src_gray.rows/(2*tan(UPPER_ANGLE_LIMIT*CV_PI/180)))) || (l[0] > (src_gray.cols/2)+(src_gray.rows/(2*tan(UPPER_ANGLE_LIMIT*CV_PI/180)))) || (l[2] < (src_gray.cols/2)-(src_gray.rows/(2*tan(UPPER_ANGLE_LIMIT*CV_PI/180)))) || (l[2] > (src_gray.cols/2)+(src_gray.rows/(2*tan(UPPER_ANGLE_LIMIT*CV_PI/180)))))
        	{
        		continue;
        	}
       }
       
       detected_lines.lines.push_back(l);
       double angle=atan2((double)(l[3]-l[1]),(double)(l[2]-l[0]))*180/CV_PI;
       
       /* Angle correction to be made due to difference between x and y axis of image and conventional 2-D space */
       if(angle > 0)
       {
       	detected_lines.angles.push_back(180-angle);
       }
       else
       {
       	detected_lines.angles.push_back(-angle);
       
       }
       /* Get length of the edge */
       double length=(double)sqrt(pow((l[3]-l[1]),2) + pow((l[2]-l[0]),2)); 
       detected_lines.lengths.push_back(length);
       /* Get intercept of the edge */
       double intercept=(double)(l[1]-tan((angle)*CV_PI/180)*l[0]);
       detected_lines.intercepts.push_back(intercept);
       /* Draw the lines on the masked image */
       cv::line(cdst, cv::Point(detected_lines.lines[j][0], detected_lines.lines[j][1]), cv::Point(detected_lines.lines[j][2], detected_lines.lines[j][3]), Scalar(255,0,0), 1, CV_AA);
       j++;
    }
    //printf("\n Number of Lines: %d---------------------", detected_lines.angles.size());
    for( size_t i = 0; i < detected_lines.angles.size(); i++ )
    {
    	/*printf("\n %d", i);
    	printf("\t From (%d, %d) to (%d, %d)", detected_lines.lines[i][0], detected_lines.lines[i][1], detected_lines.lines[i][2], detected_lines.lines[i][3]);
    	printf("\t %f", detected_lines.intercepts[i]);
    	printf("\t %f", detected_lines.angles[i]);*/
    }
    
    if(!cdst.empty())		// Display the masked grayscale image with lines drawn in color
    {
        imshow(DETECTED_LINES, cdst);        
        waitKey(1);
    }
    groups = cluster(detected_lines);	// Cluster the lines (edges) based on an edge-similarity metric
    
    fitted_lines = weighted_linear_fit(groups, cdst.rows); // Fit a straight line for each cluster
    
    curves = generate_curves(fitted_lines);	// Generate bezier curves from straight lines after connectivity analysis
    
    /* Publish only the straight edges after hough transform. Final detected lanes are published in "Publish" function */
    cv_ptr->header   = msg->header; 			   // Same timestamp and tf frame as input image
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8; // Image encoding is BGR8 to allow colored markings
    cv_ptr->image    = cdst; 				   // image container	
    hough_pub.publish(cv_ptr->toImageMsg());		   // Publish the image after converting OpenCV image to ROS image message format
    //std::cout<<"\n Edge Frame: "<<msg->header.seq;
    curves.frame_number=msg->header.seq;		   // Remembers the frame number published
  }
  
  /* Publishes the original image with detected lanes drawn on it. This function is executed when the edge detector publishes the original image */
  void Publish(const sensor_msgs::ImageConstPtr& msg)	
  {
    cv_bridge::CvImagePtr cv_ptr;		// Pointer to an OpenCV image container
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);	// Convert OpenCV image to ROS image
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());		// If not, catch an exception
      return;
    }
    /* Draw the detected lanes on the original image and display it */
    //std::cout<<"\t Original Frame: "<<msg->header.seq;
    if(curves.frame_number!=msg->header.seq)	// Check for frame correctness. If not, print an error message
    {
    	//printf("\n Invalid frame. Make sure to start this program before starting Edge detection program. \n");
    }
    else				// Otherwise, draw the detected lanes on the original image 
    {
    	for(int i=0;i<curves.drawing_points.size();i++)
    	{
 		for(int j=0;j<(curves.drawing_points[i].size()-1);j++)
 		{
 			cv::line(cv_ptr->image, cv::Point(curves.drawing_points[i][j][0], curves.drawing_points[i][j][1]), cv::Point(curves.drawing_points[i][j+1][0], curves.drawing_points[i][j+1][1]), Scalar(0,0,255), DRAWING_THICKNESS, CV_AA);
 		}
    	}
    
    	/*for(int i=0;i<fitted_lines.lines.size();i++)
    	{
    		cv::line(cv_ptr->image, cv::Point(fitted_lines.lines[i][0], fitted_lines.lines[i][1]), cv::Point(fitted_lines.lines[i][2], 		fitted_lines.lines[i][3]), Scalar(0,0,255), 1, CV_AA);
    	}*/	
        std::string temp1=folder_to_save;	// Folder to save the output frames
        std::string temp2=".jpg";		// To be saved in jpg
        char* temp3=NULL;			// Empty string
    	if(!cv_ptr->image.empty())		// If image is available
    	{
        	imshow(DETECTED_LANES_ON_ORIGINAL_IMAGE, cv_ptr->image); // Display the image  
        	temp3 = (char*)malloc(5);	// Allocate memory to save part of the filenames which will be a number
           	sprintf(temp3,"%d",msg->header.seq); // Convert the frame number to string to be inserted to the file name
        	imwrite(temp1+(std::string)temp3+temp2,cv_ptr->image); // Write the image to the specified location
        	waitKey(1);
    	}
    }
    /* Publish the frames with detected lanes */
    cv_ptr->header   = msg->header; 			   // Same timestamp and tf frame as input image
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8; // Image encoding is BGR8 to allow colored markings			
    // Output modified video stream
   final_pub.publish(cv_ptr->toImageMsg());	   	   // Publish the image after converting OpenCV image to ROS image message format
  }
};
/* ------------------------------------------------------------------------------------------------------------ */


/* ------------------------------------------------------------------------------------------------------------ */
/* Main function responsible for initialization */
int main(int argc, char** argv)		
{
  //double t1,t2;					
  //t1=clock();	
  ros::init(argc, argv, "lane_detector");	// Initialize the node
  folder_to_save=argv[1];			// Get the folder where the processed frames to be stored
  LaneDetection ic;				// Instantiate the class which initializes the subscription and publication of ROS images
  curves.frame_number=-1; //The frame currently being processed. It is initialized to -1 and indicates invalid frame number initially.
  ros::spin();					// Let the ROS core take over the execution based on incoming messages
  //t2=clock();
  //double diff = (double)((t2-t1)/CLOCKS_PER_SEC);
  //printf("\n Time: %f", diff);
  return 0;
} 
/* ------------------------------------------------------------------------------------------------------------ */
