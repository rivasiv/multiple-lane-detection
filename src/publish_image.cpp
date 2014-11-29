/* Copyright (C) Keerthi Raj Nagaraja - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Keerthi Raj Nagaraja <nagaraj1@purdue.edu>, July 2014
 */
 
/* Image Publisher */
/* Publishes images on "image_publisher/image" topic. */
/* Images Published sequentially in BGR8 format from specified folder, specified extension with specified frame rate */
/* Command Line Arguments: 
/*   folder location (Ex: /home/raj/images/) */
/*   file extension (Ex: .png) */
/*   frame rate (Ex: 15) */
/* ------------------------------------------------------------------------------------------------------------ */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <dirent.h>
#include <boost/filesystem.hpp>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <iostream>

#define BOOST_FILESYSTEM_VERSION 3
#define BOOST_FILESYSTEM_NO_DEPRECATED 

namespace fs = boost::filesystem;

/* ------------------------------------------------------------------------------------------------------------ */
/* Function to get all the filenames with a specified extension */
/* Uses boost filesystem library */
/* Arguments: */
/*   path: the folder location */
/*   extension: the file extension */
/*   filenames: pointer to a vector of strings to store the filenames */
void GetFilenames(char* path, char* extension, std::vector<std::string> &filenames)
{
    fs::path directory(path);				//Set the path
    fs::directory_iterator iter(directory), end;	//Set the ierator
    
    for(;iter != end; ++iter)				// Loop through all the files in the directory
    {
        if (iter->path().extension() == extension)	// If the extension matches, then store the filename in the vector
        {
            filenames.push_back(iter->path().filename().string());
	}    
    }	
}
/* ------------------------------------------------------------------------------------------------------------ */




/* ------------------------------------------------------------------------------------------------------------ */
/* Main function*/
/* Initializes a node and image publisher*/
/* Gets all the files to be published, reads each one of them and publishes them sequentially */
/* The publishing is repeated from the beginning if all the files are published */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;				// ROS node handler
  image_transport::ImageTransport it(nh);	// Image transporter 
  image_transport::Publisher pub = it.advertise("image_publisher/image", 1); // Image publisher
  
  std::vector<std::string> filenames;		// Vector to hold filenames of the images to be published
  std::string temp="/";				// A string variable 
  int count=0;					// An integer variable for counting the number of files
  
  GetFilenames(argv[1], argv[2], filenames);	// Get all the files which have specified file extension
  std::sort( filenames.begin(), filenames.end() );	// Sort them alphabetically
  std::cout<<"\n"<<filenames.size()<<" Frames found\n"; // Print a message indicating the number of files found
  
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage); // pointer to OpenCV image container
  ros::Rate loop_rate(atoi(argv[3]));		// ROS loop rate which decides the frame rate for publishing
  
  while (nh.ok()) 				// Until the ROS node is being active 
  {
    cv_ptr->image = cv::imread(argv[1]+temp+filenames[count], CV_LOAD_IMAGE_COLOR);	// Read an image and copy it to OpenCV image container
    if ((cv_ptr->image.rows > 0) && (cv_ptr->image.cols > 0))
    {
    	cv_ptr->encoding = "bgr8";			// Specify encoding as BGR8
    	pub.publish(cv_ptr->toImageMsg());		// Convert OpenCV image to ROS image format and publish it
    	ros::spinOnce();				// The ROS core calls all the callbacks waiting at this instance
    	loop_rate.sleep();				// No more callbacks occur after this
    	std::cout<<count<<"\n";			// Print the frame number published
     }
     count++;					// Increase the frame number
     if(count==filenames.size())		// If all files are published, reset the counter and repeat publishing from the start
     {		
   	count=0;					
     }	
  }
}
/* ------------------------------------------------------------------------------------------------------------ */
