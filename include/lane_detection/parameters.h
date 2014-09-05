/* Copyright (C) Keerthi Raj Nagaraja - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Keerthi Raj Nagaraja <nagaraj1@purdue.edu>, July 2014
 */
 
#ifndef PARAMETERS_H
#define PARAMETERS_H

//#define CALTECH_DATASET

#define MASK_REGION_CONST_1 0.4		//40% of the image height	
#define MASK_REGION_CONST_2 0.5 	//50% of the image height
#define MASK_REGION_CONST_3 0.5		//50% of the image width
#define MASK_REGION_CONST_4 0.75	//75% of the image height - Only for Caltech dataset
 
#define DRAWING_REGION 0.5 		//50% of the image height
#define DRAWING_THICKNESS 2

#define HT_VOTES_THRESHOLD 15
#define HT_MIN_LINE_LENGTH 15
#define HT_MAX_DIST_BW_LINES 1

#define LOWER_ANGLE_LIMIT 10
#define UPPER_ANGLE_LIMIT 85

#define NUMBER_OF_FEATURES_FOR_CLUSTERING 2
#define WEIGHT_FOR_PERP_DIST 0.75
#define WEIGHT_FOR_ANGLE 1.25
#define TREE_PRUNING_DIST_THRESHOLD 40

#define THRESHOLD_WEIGHT_FOR_FITTING 0
#define MAX_ANGLE_DEVIATION_AFTER_FITTING 5

#define ANGLE_DIFF_FACTOR_1 20
#define ANGLE_DIFF_FACTOR_2 60
#define ANGLE_DIFF_FACTOR_3 90

#define CURVE_RESOLUTION 0.05

struct detected_lines			//Structure to store intial detected lines after hough transform
{
  std::vector<cv::Vec4i> lines;		// Vector of set of two points at the ends of the lines
  std::vector<double> angles;		// Slopes of the lines	
  std::vector<double> lengths;		// Lengths of the lines
  std::vector<double> intercepts;	// y-intercepts of the lines
};

struct clustered_lines			// Structure to store the classified lines as groups. One struct variable stores one group of lines
{
  std::vector<cv::Vec4i> lines;		// Vector of set of two points at the ends of the lines
  std::vector<double> angles;		// Slopes of the lines	
  std::vector<double> lengths;		// Lengths of the lines
  std::vector<double> weights;		// Importance weights of each line in a group of lines.
  double mean_angle;
  double clustering_distance;
};

struct fitted_lines			// Structure to store the fitted lines 
{
  std::vector<cv::Vec2f> slopes_intercepts; // Slopes and intercepts of the lines
  std::vector<cv::Vec4i> lines;	// vector of pair of points to define the ends of lines
  std::vector<double> angles; 	// Angles (slopes) of the lines
};

struct bezier_curves
{
 std::vector< std::vector<cv::Vec2f> > control_points;
 std::vector< std::vector<cv::Vec2f> > drawing_points;
 unsigned int frame_number;
};

struct node
{
  int line_number;
  int connections;
  int parents;
  double* cost_to_connect;
  node** connectors;
  node** parent_connectors;
  double min_cost_to_child;
  node* min_path_to_child;
  double min_cost_from_parent;
  node* min_path_from_parent;
};

#endif
