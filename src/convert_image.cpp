/* Copyright (C) Keerthi Raj Nagaraja - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Keerthi Raj Nagaraja <nagaraj1@purdue.edu>, July 2014
 */
 
/* Image Converter */
/* Converts all images in a folder from one extension to another. */
/* Converted Images are saved in the same folder as the original image with same filename */
/* Command Line Arguments: 
/*   folder location (Ex: /home/raj/images/) */
/*   original file extension (Ex: .png) */
/*   required file extension (Ex: .jpg) */
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
/* Function to get all the filenames with original file extension */
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
/* Initializes a node */
/* Gets all the files to be converted, reads each one of them and converts them to the required file type */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");	
  
  std::vector<std::string> filenames;		// Vector to hold filenames of the images
  std::string temp="/";				// A string variable 
  
  GetFilenames(argv[1], argv[2], filenames);		// Get all the files which have original file extension
  std::sort( filenames.begin(), filenames.end() );	// Sort them alphabetically
  std::cout<<"\n"<<filenames.size()<<" Files found\n";  // Print a message indicating the number of files found
  
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage); // pointer to OpenCV image container
  
  for(int i=0;i<filenames.size();i++)			// Loop through all the files to be converted
  {
    cv_ptr->image = cv::imread(argv[1]+temp+filenames[i], CV_LOAD_IMAGE_COLOR);	// Read the image into a OpenCV image container
    cv_ptr->encoding = "bgr8";				// Specify encoding as BGR8
    int j=0;						// A temporary character indexing variable
    while(filenames[i][j]!=0)				// Search for the end of a filename (Null Character)
    {
    	j++;						
    }
    j=j-strlen(argv[2]);				// Assign the character index to the start of the file extension
    int k=0;						// Another temporary character indexing variable
    while(argv[3][k]!=0)				// Copy the required extension to the end of the filename
    {
        filenames[i][j+k]=argv[3][k];
    	k++;
    }
    filenames[i][j+k]=0;
    imwrite(argv[1]+temp+filenames[i],cv_ptr->image); 	// Save the image back to the folder with a filename that has required extension
    printf("\n%d",i);					// Print the image number which just got converted 
  }
}
/* ------------------------------------------------------------------------------------------------------------ */
