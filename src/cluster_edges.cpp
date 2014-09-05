/* Copyright (C) Keerthi Raj Nagaraja - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Keerthi Raj Nagaraja <nagaraj1@purdue.edu>, July 2014
 */
 
/* This source file has functions which are used for clustering the edges and fitting a straight line for each cluster */
/* These functions are called by the functions in main.cpp */
/* ------------------------------------------------------------------------------------------------------------ */ 
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <lane_detection/parameters.h>
#include <lane_detection/cluster_edges.hpp>
#include <lane_detection/cluster_algorithm.h>

using namespace cv;

/* ------------------------------------------------------------------------------------------------------------ */
/* This function takes in the detected straight edges as input and clusters them into multiple clusters based on an edge-similarity metric*/
std::vector<struct clustered_lines> cluster(struct detected_lines input_lines)
{
 std::vector<struct clustered_lines> groups;	// vector of struct variables, each struct stores lines which belong to one cluster 
 struct clustered_lines temp;			// local temporary struct to store a cluster
 std::vector<int> locations_to_delete;		// A generic integer vector which is used to store the index of some of the elements to be deleted in any vector based datastructure  
 std::vector<int> lines_in_group;		// Temporary vector to store indices of the lines which belong to same cluster
 
 Node *tree;					// A tree structure pointer used to store the result of hierarchical clustering
 int nrows=input_lines.lines.size();		// Number of lines (edges) are number of elements to be clustered which indicate the rows 							in the dataset
 int ncolumns=NUMBER_OF_FEATURES_FOR_CLUSTERING; // Number of features for each element indicate the number of columns of the dataset
 double weight_for_pd = WEIGHT_FOR_PERP_DIST; 	 // Weightage given for the feature "Perpendicular Distance"
 double weight_for_angle = WEIGHT_FOR_ANGLE;	 // Weightage given for the feature "Angle difference"
 
 double **distmatrix;				// Matrix to store distance between each of the lines(edges)
 /* Allocate the memory for the matrix */
 if((distmatrix = new double*[nrows])==NULL)	
 {
 	printf("\n Memory issue");
 	return groups;
 }
 for(int i=0;i<nrows;i++)
 {
 	if((distmatrix[i]=new double[nrows])==NULL)
 	{
 		printf("\n Memory issue");
 		return groups;
 	}
 }
 
 /* Populate the distance matrix by calculating the edge-similarity metric (distance) between each of the edges(lines)*/
 double distance;
 for(int i=0; i<nrows; i++)
 {
  for(int j=0; j<nrows; j++)
  {
  	if(i==j)			// Distance is 0 when both lines are same
  	{
  		distmatrix[i][j]=0;	
  	}	
  	else
  	{		/* Calculate the perpendicular distance between lines using the proposed formula */
  			if(input_lines.angles[i]<90)
  			{
  				distance = abs((tan(-input_lines.angles[i]*CV_PI/180)*input_lines.lines[j][0])-input_lines.lines[j][1]+input_lines.intercepts[i]) + abs((tan(-input_lines.angles[i]*CV_PI/180)*input_lines.lines[j][2])-input_lines.lines[j][3]+input_lines.intercepts[i]);
  				distance = distance/(2*sqrt(pow(tan(-input_lines.angles[i]*CV_PI/180),2)+1));
  				
			}
			else
			{
				distance = abs((tan((180-input_lines.angles[i])*CV_PI/180)*input_lines.lines[j][0])-input_lines.lines[j][1]+input_lines.intercepts[i]) + abs((tan((180-input_lines.angles[i])*CV_PI/180)*input_lines.lines[j][2])-input_lines.lines[j][3]+input_lines.intercepts[i]);
  				distance = distance/(2*sqrt(pow(tan((180-input_lines.angles[i])*CV_PI/180),2)+1));			
			}
			
			/* Calculate the total distance after considering angle difference and weightages */
			distmatrix[i][j]=weight_for_angle*abs(input_lines.angles[i]-input_lines.angles[j])+weight_for_pd*distance;
			if(i>j)	// Make the distance matrix symmetric by considering only the maximum among two distances between two lines
			{
				distmatrix[i][j]=max(distmatrix[i][j], distmatrix[j][i]);
				distmatrix[j][i]=max(distmatrix[i][j], distmatrix[j][i]);
			}
  	}
  }
 } 
  
 
 /* Run the hierarchical clustering algorithm using the function below */  
 /*Node* treecluster (int nrows, int ncolumns, double** data, int** mask,
  double weight[], int transpose, char dist, char method, double** distmatrix)*/
 tree = treecluster(nrows, ncolumns, NULL, NULL, NULL, 0, 'e', 'a' , distmatrix);
 
 if(tree==NULL)				// If clustering failed, print an error message and return
 {
 	printf("\n clustering failed");
 	return groups;
 }
 else				// If clustering succeeded, extract the tree structure from the result of clustering
 {
 	int pruning_index;	// Integer representing where to prune the tree for reasonable clustering
	std::vector<int> nodes_to_skip;	// An integer vector to store the indices of nodes in the tree which shall be skipped while extracting the hierarchical clustering tree structure
 	//printf("\n ----------%d Elements-----------------------", nrows);
 	/* This loop searches for pruning index while recognizing the nodes to be skipped later when building the tree */
 	for(pruning_index=0;pruning_index<(nrows-1);pruning_index++)		
 	{
 		if(tree[pruning_index].distance>TREE_PRUNING_DIST_THRESHOLD) // Pruning index is decided based on distance threshold
 		{
 			break;
 		}
 		/* If the tree nodes are negative values, then remember them to skip as they indicate that the node is a group of individual lines and will be extracted as individual lines later */
 		if(tree[pruning_index].left<0)		
 		{
 			nodes_to_skip.push_back((-tree[pruning_index].left)-1);
 		}
 		if(tree[pruning_index].right<0)	
 		{
 			nodes_to_skip.push_back((-tree[pruning_index].right)-1);
 		}
 		//printf("\n (%d, %d) : %f", tree[pruning_index].left, tree[pruning_index].right, tree[pruning_index].distance);
 	}
 	
 	int flag, all_positive, temp_index;	// Temporary variables used while extracting the tree structure and hence clusters
 	/* This loop iteratively extracts different clusters from the hierarchical tree structure */
 	for(int i=(pruning_index-1);i>=0;i--)	
 	{
 		flag=0;
 		/* If a node to be skipped is encountered, then just continue */ 
 		for(int j=0;j<nodes_to_skip.size();j++)		
 		{
 			if(i==nodes_to_skip[j])
 			{
 				flag=1;
 				break;
 			}
 		}	
 		if(flag==1)
 		{
 			continue;
 		}
 		
 		/* Othwerise, extract the lines(edges) which belong to that node */
 		lines_in_group.clear();				// Clear the temporary storage vector
 		
 		lines_in_group.push_back(tree[i].left);// Store the initial line indices (some of them can be negative and indicate group of lines 		
 		lines_in_group.push_back(tree[i].right);
 		
 		/* Iteratively replace all negative indices (group of lines) by their individual line indices within the group */
 		do
 		{
 			all_positive=1;
 			for(int k=0;k<lines_in_group.size();k++)	
 			{
 				if(lines_in_group[k]<0)
 				{
 					temp_index=-lines_in_group[k]-1;
 					lines_in_group[k]=tree[temp_index].left;
 					lines_in_group.push_back(tree[temp_index].right);
 					break;
 				}
 			}
 			for(int k=0;k<lines_in_group.size();k++)
 			{
 				if(lines_in_group[k]<0)
 				{
 					all_positive=0;
 					break;
 				}
 			}
 		}while(all_positive==0);
 		
 		/* At this point, "lines_in_group" vector contains individual line(edge) indices which belong to one cluster */ 
 		/* Extract the information pertaining to these edges and store them in "groups" structure */
 		temp.lines.clear();	// Clear local "temp" vector
    		temp.angles.clear();
    		temp.lengths.clear();
    		temp.weights.clear();
    		double sum_of_weights=0;
    		double mean_angle=0;
    		/* This loop extracts the information (points, angles, lengths) of each edge which belong to same cluster */
 		for(int k=0;k<lines_in_group.size();k++)
 		{
 			temp.lines.push_back(input_lines.lines[lines_in_group[k]]);
    			temp.angles.push_back(input_lines.angles[lines_in_group[k]]);
    			temp.lengths.push_back(input_lines.lengths[lines_in_group[k]]);
    			temp.weights.push_back(input_lines.lengths[lines_in_group[k]]);
    			sum_of_weights+=input_lines.lengths[lines_in_group[k]];
    			mean_angle+=input_lines.angles[lines_in_group[k]];
 		}
 		/* Calculate the relative weights(importance) of each edge within the cluster */
 		for(int k=0;k<temp.lines.size();k++)
 		{
 			temp.weights[k]=temp.weights[k]*100/sum_of_weights;
 		}
 		/* Get the clustering distance, mean angle of each cluster and store the cluster in "groups" data structure */
 		if(temp.lines.size()!=0)
 		{
 			temp.clustering_distance= tree[i].distance;
 			temp.mean_angle=mean_angle/temp.lines.size();
 			groups.push_back(temp);	
 		}
 	}
 	
 
 	/*printf("\nGrouped Lines:-----------------------------------");
 	printf("\nNumber of groups: %d", (int)groups.size());
 	for(int i=0;i<groups.size();i++)
 	{
 	  for(int j=0;j<groups[i].lines.size();j++)
 	  {
 	  	printf("\n Line %d:", j);
    		printf("\t %f",groups[i].angles[j]);
    		printf("\t %f",groups[i].lengths[j]);
    		//printf("\t From (%d, %d) to (%d, %d)", groups[i].lines[j][0], groups[i].lines[j][1], groups[i].lines[j][2], groups[i].lines[j][3]);
    		printf("\t %f", groups[i].weights[j]);
 	  }
 	  printf("\n-------- %f ------- %f\n", groups[i].clustering_distance, groups[i].mean_angle);
 	}*/
 }
 /* de-allocate all the memory reserved for distance matrix */
 for(int i=0;i<nrows;i++)
 {
 	delete[] distmatrix[i];
 }
 delete[] distmatrix;
 
 
 return groups;	// Return the data-structure which contains all clusters and their respective edges
}
/* ------------------------------------------------------------------------------------------------------------ */



/* ------------------------------------------------------------------------------------------------------------ */
/* This function fits a straight line(y=mx+c) to each cluster of edges using the weighted least squares fit */
/* Two end points from each edge of a cluster is grouped together and multiple "y1 = m*x1 + c" equations are represented using */
/* A*X=Y where, X=[m c]', Y=[y1 y2 ...yn]' and A is a Rank 2 matrix of dimension nx2. */
/* The aim is to calculate vector X which denotes a straight line for each cluster using X=((A'A)^-1)A'Y*/
struct fitted_lines weighted_linear_fit(std::vector<struct clustered_lines> groups, int image_y_length)
{
   struct fitted_lines fitted_lines;			// Structure to store the fitted lines
   std::vector<cv::Vec2f> A; 				// Matrix A
   std::vector<float> y;     				// Vector Y
   boost::array<double, 4> ATA; 			// A transpose A
   boost::array<double, 4> inv_ATA; 			// Inverse of A transpose A
   std::vector<cv::Vec2f> Psuedo_inv_A; 		// Moore-Penrose Psuedo inverse of A
   std::vector<int> lines_to_delete; 			// An integer vector to store indices of fitted lines to be discared
   
   /* This loop fits a line for each cluster in one iteration */
   for(int i=0;i<groups.size();i++)
   {
     /* Initialize few data-structures */
     A.clear();	
     y.clear();
     Psuedo_inv_A.clear();
     double slope=0, intercept=0;
     int max_y=0, min_y=0;
     
     for(int j=0;j<groups[i].lines.size();j++) // Calculate A and Y along with max_y and min_y in a group.
     {
        if(groups[i].weights[j] > THRESHOLD_WEIGHT_FOR_FITTING)	// Consider only lines with importance weight greater than 10
        {
        	if(y.size()==0)
        	{
        		max_y=groups[i].lines[j][1];
        		min_y=groups[i].lines[j][1];
        	}
        	else
        	{
        		if(groups[i].lines[j][1]>max_y)
        		{
        			max_y=groups[i].lines[j][1];
        		}
        		if(groups[i].lines[j][1]<min_y)
        		{
        			min_y=groups[i].lines[j][1];
        		}
        	}
     		A.push_back(Vec2f(groups[i].lines[j][0]*groups[i].weights[j]/100, groups[i].weights[j]/100));
     		A.push_back(Vec2f(groups[i].lines[j][2]*groups[i].weights[j]/100, groups[i].weights[j]/100));
     		y.push_back(groups[i].lines[j][1]*groups[i].weights[j]/100);
     		y.push_back(groups[i].lines[j][3]*groups[i].weights[j]/100);
     		if(groups[i].lines[j][3]>max_y)
        	{
        		max_y=groups[i].lines[j][3];
        	}
        	if(groups[i].lines[j][3]<min_y)
        	{
        		min_y=groups[i].lines[j][3];
        	}
     	}
     }	
     /* Calculate A transpose A */
     ATA[0]=0;ATA[1]=0;ATA[2]=0;ATA[3]=0;
     for(int k=0;k<A.size();k++)	
     {
     	ATA[0]+=A[k][0]*A[k][0];
     	ATA[1]+=A[k][1]*A[k][0];
     	ATA[2]+=A[k][0]*A[k][1];
     	ATA[3]+=A[k][1]*A[k][1];
     }	
     /* Calculate Inverse of A transpose A */
     double det_ATA=(ATA[0]*ATA[3])-(ATA[1]*ATA[2]); 
     inv_ATA[0]=ATA[3]/det_ATA;
     inv_ATA[1]=-ATA[1]/det_ATA;
     inv_ATA[2]=-ATA[2]/det_ATA;
     inv_ATA[3]=ATA[0]/det_ATA;
     
     /* Calculate Moore-Penrose Psuedo inverse of A transpose A */
     for(int k=0;k<A.size();k++)	
     {
     	Psuedo_inv_A.push_back(Vec2f(inv_ATA[0]*A[k][0]+inv_ATA[1]*A[k][1] , inv_ATA[2]*A[k][0]+inv_ATA[3]*A[k][1]));
     }
     /* Calculate X (slope and intercept) of the fitted line for a cluster */
     for(int k=0;k<A.size();k++)	
     {
     	slope+=Psuedo_inv_A[k][0]*y[k];
     	intercept+=Psuedo_inv_A[k][1]*y[k];
     }
     
     /* Calculate the slope of the fitted line */
     double angle=atan(slope)*180/CV_PI;
     if(angle<0)
     {
     	angle=angle*-1;
     }
     else
     {
     	angle=180-angle;
     }
     /* Store the information about fitted line in the "fitted_lines" data-structure */
     fitted_lines.slopes_intercepts.push_back(Vec2f(slope,intercept));
     fitted_lines.angles.push_back(angle);
     fitted_lines.lines.push_back(Vec4i(round((max_y-fitted_lines.slopes_intercepts[i][1])/fitted_lines.slopes_intercepts[i][0]), max_y, round((min_y-fitted_lines.slopes_intercepts[i][1])/fitted_lines.slopes_intercepts[i][0]), min_y));
   }
   
   /* If the fitted line's slope is different from the cluster's mean slope, then delete the fitted line because its a noisy group */
   lines_to_delete.clear();
   for(int i=0;i<fitted_lines.lines.size();i++)
   {
   	if(abs(fitted_lines.angles[i]-groups[i].mean_angle) > MAX_ANGLE_DEVIATION_AFTER_FITTING)
   	{
   		lines_to_delete.push_back(i);
   	}
   }
   for(int i=0;i<lines_to_delete.size();i++)
   {
   	int n=lines_to_delete[i]-i;
 	fitted_lines.lines.erase(fitted_lines.lines.begin()+n);
 	fitted_lines.slopes_intercepts.erase(fitted_lines.slopes_intercepts.begin()+n);
 	fitted_lines.angles.erase(fitted_lines.angles.begin()+n);
   }
   
   /* Re-order the points which represents the fitted lines so that the first point is lower than second point in the image */
   for(int i=0;i<fitted_lines.lines.size();i++) 
   {
   	int swap;
   	Vec4i l = fitted_lines.lines[i];
        if(l[1]<l[3])
        {
       		swap = l[0];
       		l[0]=l[2];
       		l[2]=swap;
       		swap=l[1];
       		l[1]=l[3];
       		l[3]=swap;
       }
       fitted_lines.lines[i]=l;
   }
   
   /*for(int i=0;i<fitted_lines.lines.size();i++)
   {
   	printf("\n Fitted Line %d", i);
   	printf("\t %f",fitted_lines.angles[i]);
   	printf("\t %f",fitted_lines.slopes_intercepts[i][1]);
   	printf("\t From (%d, %d) to (%d, %d)", fitted_lines.lines[i][0], fitted_lines.lines[i][1], fitted_lines.lines[i][2], fitted_lines.lines[i][3]);
   }	*/
   
 return fitted_lines;	// Return the data-structure "fitted_lines" which has all the information about lines fitted to each cluster
}
/* ------------------------------------------------------------------------------------------------------------ */
