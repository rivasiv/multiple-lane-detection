/* Copyright (C) Keerthi Raj Nagaraja - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Keerthi Raj Nagaraja <nagaraj1@purdue.edu>, July 2014
 */
 
/* This source file has functions which are used for connectivity-tree generation and bezier curve generation for lane representation */
/* These functions are called by the functions in main.cpp */
/* ------------------------------------------------------------------------------------------------------------ */  
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>

#include <lane_detection/bezier_curve.hpp>
#include <lane_detection/parameters.h>


/* ------------------------------------------------------------------------------------------------------------ */ 
/* This function takes in the straight lines, each of which represents a cluster. The cost of connectivity is assigned between each*/
/* pair of lines. Then the optimal group of lines which represent a lane are found out by dynamic programming. The optimal group of 
/* lines are used to obtain control points which are necessary for generating bezier curves. */
/* The bezier curves are generated using De Casteljau's algorithm. Each bezier curve represents a lane marker */
struct bezier_curves generate_curves(struct fitted_lines input_lines)
{
 struct bezier_curves curves;			// Data-structure to store the information about the generated bezier curves
 cv::Vec2f **IP_Matrix;				// Matrix(nxn) to store the intersection point in bezier parameter 't' between pair of 							lines (clusters)
 int nrows = input_lines.lines.size();		// The number of lines(clusters) are the number of rows(n) of the matrix 
 cv::Vec2f t;					// A temporary 2-D vector to store the intersection point of any two lines in bezier 							parameter representation
 
 
 struct node root; 				// Structure to represent a node in the connectivity tree. Root node is declared here.
 /* Few initializations for the root node */
 root.line_number=-1; root.connections=0; root.cost_to_connect=NULL; root.parents=0; root.connectors=NULL; root.parent_connectors=NULL;
 root.min_cost_to_child=0; root.min_path_to_child=NULL; root.min_cost_from_parent=0; root.min_path_from_parent=NULL;
 
 struct node nodes[nrows];			// Declare nodes equal to the number of lines(clusters)
 
 std::vector< std::vector<int> > group_of_lines;	// Data-structure to store group of lines which belong to the same lane
 std::vector<cv::Vec2f> temp_control_points;		// temporary data-structure to store control points for a bezier curve
 std::vector<cv::Vec2f> temp_drawing_points;		// temporary data-structure to store sampled points to draw a bezier curve

 /* Allocate memory for the Intersection Point (IP) matrix while initializing all the nodes of the connectivity tree */
 if((IP_Matrix = new cv::Vec2f*[nrows])==NULL)		
 {
 	printf("\n Memory issue");
 	return curves;
 }
 for(int i=0;i<nrows;i++)
 {
        if((IP_Matrix[i]=new cv::Vec2f[nrows])==NULL)
        {
 		printf("\n Memory issue");
 		return curves;
 	}
 	
 	nodes[i].line_number=i;
 	nodes[i].connections=0;
 	nodes[i].cost_to_connect=NULL;
 	nodes[i].parents=0;
 	nodes[i].connectors=NULL;
 	nodes[i].parent_connectors=NULL;
 	nodes[i].min_cost_to_child=0;
 	nodes[i].min_path_to_child=NULL;
 	nodes[i].min_cost_from_parent=0;
 	nodes[i].min_path_from_parent=NULL;
 }
 /* few temporary variables */
 int parent, child;	
 double temp_cost;
 
 /* This loop calculates cost of connectivity between pair of lines(nodes) and link each node(line) with its parents and children and iteratively builds the connectivity-tree */
 for(int i=0; i<nrows; i++)	// Loop through for all the nodes(lines)
 {
  for(int j=0; j<i; j++)	// Loop through all the nodes(lines) up until the current node of outer loop
  {	
  		/* Calculate intersection point between two lines in terms of bezier parameter 't' representation */
  		t[0]=((-input_lines.slopes_intercepts[j][1]+input_lines.slopes_intercepts[i][1])/(tan(input_lines.angles[i]*CV_PI/180)-tan(input_lines.angles[j]*CV_PI/180)) - input_lines.lines[i][0])/(input_lines.lines[i][2]-input_lines.lines[i][0]);
  		t[1]=((-input_lines.slopes_intercepts[i][1]+input_lines.slopes_intercepts[j][1])/(tan(input_lines.angles[j]*CV_PI/180)-tan(input_lines.angles[i]*CV_PI/180)) - input_lines.lines[j][0])/(input_lines.lines[j][2]-input_lines.lines[j][0]);
  		IP_Matrix[i][j]=t;
  		
  		if(((t[0]<0)&&(t[1]<0))||((t[0]>1)&&(t[1]>1))) // If converging or diverging lines, discard them
  	   	{
  	   		continue;
  	   	}
  	   	/* Otherwise, find cost of connectivity between two lines(nodes) based on intersection point and a factor of angle difference between the lines */
  	   	else					
  	   	{
  	   		//printf("\n (%d, %d)", i, j);
	   		//printf("\t (%f, %f)", (float)(t[0]), (float)(t[1]));
	   		//printf("\t (%f, %f)", input_lines.angles[i], input_lines.angles[j]);
  	   		if(((t[0]<0)&&(t[1]>1))||((t[0]>1)&&(t[1]<0)))
  	   		{
  	   			if(t[1]>t[0])
  	   			{
  	   				parent = j; child = i;	
  	   				temp_cost=(t[1]-1-t[0])+(abs(input_lines.angles[i]-input_lines.angles[j])/ANGLE_DIFF_FACTOR_3);
  	   			}
  	   			else
  	   			{
  	   				parent = i; child = j;
  	   				temp_cost=(t[0]-1-t[1])+(abs(input_lines.angles[i]-input_lines.angles[j])/ANGLE_DIFF_FACTOR_3);
  	   			}	
  	   		}
  	   		else
  	   		{
  	   			if((input_lines.lines[i][1]>input_lines.lines[j][1])&&(input_lines.lines[i][1]>input_lines.lines[j][3])&&(input_lines.lines[j][3]<input_lines.lines[i][3]))
  	   			{
  	   				parent = i; child = j;
  	   				if(((t[0]<0)&&(t[1]>0)&&(t[1]<1))||((t[1]<0)&&(t[0]>0)&&(t[0]<1)))
  	   				{
  	   					temp_cost=(1-t[0]-t[1])+(abs(input_lines.angles[i]-input_lines.angles[j])/ANGLE_DIFF_FACTOR_2);
  	   				}
  	   				else if(((t[0]>1)&&(t[1]>0)&&(t[1]<1))||((t[1]>1)&&(t[0]>0)&&(t[0]<1)))
  	   				{
  	   					temp_cost=(t[0]-1+t[1])+(abs(input_lines.angles[i]-input_lines.angles[j])/ANGLE_DIFF_FACTOR_2);
  	   				}
  	   				else
  	   				{
  	   					temp_cost=(1-t[0]+t[1])+(abs(input_lines.angles[i]-input_lines.angles[j])/ANGLE_DIFF_FACTOR_1);
  	   				}
  	   			}
  	   			else if((input_lines.lines[j][1]>input_lines.lines[i][1])&&(input_lines.lines[j][1]>input_lines.lines[i][3])&&(input_lines.lines[i][3]<input_lines.lines[j][3]))
  	   			{
  	   				parent = j; child = i;
  	   				if(((t[0]<0)&&(t[1]>0)&&(t[1]<1))||((t[1]<0)&&(t[0]>0)&&(t[0]<1)))
  	   				{
  	   					temp_cost=(1-t[1]-t[0])+(abs(input_lines.angles[i]-input_lines.angles[j])/ANGLE_DIFF_FACTOR_2);
  	   				}
  	   				else if(((t[0]>1)&&(t[1]>0)&&(t[1]<1))||((t[1]>1)&&(t[0]>0)&&(t[0]<1)))
  	   				{
  	   					temp_cost=(t[1]-1+t[0])+(abs(input_lines.angles[i]-input_lines.angles[j])/ANGLE_DIFF_FACTOR_2);
  	   				}
  	   				else
  	   				{
  	   					temp_cost=(1-t[1]+t[0])+(abs(input_lines.angles[i]-input_lines.angles[j])/ANGLE_DIFF_FACTOR_1);
  	   				}
  	   			}
  	   			else
  	   			{
  	   				continue;
  	   			}
  	   		}	
  	   		/* link parents and children */
  	   		nodes[parent].connectors=(node**)realloc(nodes[parent].connectors, sizeof(node*)*(nodes[parent].connections+1));	
  	   		nodes[parent].cost_to_connect=(double*)realloc(nodes[parent].cost_to_connect, sizeof(double)*(nodes[parent].connections+1));
  	   		nodes[child].parent_connectors=(node**)realloc(nodes[child].parent_connectors, sizeof(node*)*(nodes[child].parents+1));
  	   		/* If any memory error while linking nodes, throw an error and exit from the function */
  	   		if((nodes[parent].connectors == NULL)||(nodes[parent].cost_to_connect == NULL)||(nodes[child].parent_connectors == NULL))
  	   		{
  	   			printf("\n memory error");
  	   			return curves;
  	   		}	
  	   		/* Update parent and child nodes to reflect the existing state of the tree */
  	   		nodes[parent].connectors[nodes[parent].connections]=&(nodes[child]);
  	   		nodes[parent].cost_to_connect[nodes[parent].connections]=temp_cost;
  	   		nodes[child].parent_connectors[nodes[child].parents]=&(nodes[parent]);
  	   		nodes[parent].connections++;
  			nodes[child].parents++;
	   		//printf("\n (%d -> %d) : %f", parent, child, temp_cost);
  	   	}
   }
  } 	
  /* This loop finds out which is the root node(line) among all the nodes and updates few variables of the root node to reflect the complete structure of the tree */
 for(int i=0; i<nrows; i++)
 {
 	if(nodes[i].parents==0)
 	{
 		root.connectors=(node**)realloc(root.connectors, sizeof(node*)*(root.connections+1));
 		root.cost_to_connect=(double*)realloc(root.cost_to_connect, sizeof(double)*(root.connections+1));
 		if((root.connectors == NULL)||(root.cost_to_connect == NULL))
 		{
 			printf("\n memory error");
  	   		break;
 		}
 		root.connectors[root.connections]=&(nodes[i]);	
 		root.cost_to_connect[root.connections]=-100;
 		root.connections++;
 	}
 } 
 
 /* At this point, the connectivity tree is fully grown and populated */
 /* The loop below recursively finds the minimum-cost children for each of the nodes in the tree except the root node */
 for(int i=0;i<root.connections;i++)
 {
 	//printf("\n Branch:");
 	traverse_tree(root.connectors[i]);	
 	//printf("\t %d",traverse_tree(root.connectors[i]));
 	find_min_cost_among_children(root.connectors[i]);
 	//printf("\t %f", find_min_cost_among_children(root.connectors[i]));
 }
 /* The loop below recursively finds the minimum-cost parent for each of the nodes starting from the leaf nodes */
 for(int i=0;i<nrows;i++)
 {
 	if(nodes[i].connectors==0)
 	{
 		find_min_cost_among_parents(&(nodes[i]));
 		//printf("\n %f", find_min_cost_among_parents(&(nodes[i])));
 	}
 }
 
 /*for(int i=0;i<nrows;i++)
 {
 	printf("\n Node %d:",nodes[i].line_number);
 	if(nodes[i].min_path_from_parent == NULL)
 	{
 		printf(" best parent: None");
 	}
 	else
 	{
 		printf(" best parent: %d",nodes[i].min_path_from_parent->line_number);
 	}
 	if(nodes[i].min_path_to_child == NULL)
 	{
 		printf(" best child: None");
 	}
 	else
 	{
 		printf(" best child: %d",nodes[i].min_path_to_child->line_number);
 	}
 }*/
 
 std::vector<int> temp;	// A vector of integers to store the indices of lines which make up a group (lane marker)
 /* This loop extracts the minimum-cost path from each of the children of the root node to any reachable leaf and each such path traverses through a set of nodes(lines) which make up a lane marker. Such group of lines are stored in the data-structure "group_of_lines" */
 for(int i=0; i<root.connections; i++)
 {
 	temp.clear();
 	temp.push_back(root.connectors[i]->line_number);
 	extract_lines(root.connectors[i], &temp);
 	group_of_lines.push_back(temp);
 	//printf("\n Lane %d: ", i);
 	for(int j=0;j<group_of_lines[i].size();j++)
 	{
 		//printf("%d -> ", group_of_lines[i][j]);
 	}
 }
 
 /* This loop generates a bezier curve for each group of lines by first extracting the control points from group of lines and then applying De Casteljau's algorithm */
 for(int i=0;i<group_of_lines.size();i++)
 {
  temp_control_points.clear();		// Clear the temporary data-structure to store control points of a group (curve)
  for(int j=0;j<group_of_lines[i].size();j++)
  {
  	/* Extract two control points which are the end-points of a line within the group of lines */
  	temp_control_points.push_back(cv::Vec2f(input_lines.lines[group_of_lines[i][j]][0],input_lines.lines[group_of_lines[i][j]][1]));
  	temp_control_points.push_back(cv::Vec2f(input_lines.lines[group_of_lines[i][j]][2],input_lines.lines[group_of_lines[i][j]][3]));
  	/* For every line except the last line the group, check if the connectivity is because of an intersection point which lies outside both lines. If that's true, consider the intersection point as an extra control point for more smoother bezier curve */
  	if(j<(group_of_lines[i].size()-1))
  	{
  		/* Get the intersection point from the Intersection Point Matrix */
  		if(group_of_lines[i][j]>group_of_lines[i][j+1])
  		{
  			t=IP_Matrix[group_of_lines[i][j]][group_of_lines[i][j+1]];
  		}	
  		else
  		{
  			t=IP_Matrix[group_of_lines[i][j+1]][group_of_lines[i][j]];
  		}
  		/* If the connectivity is because of an intersection point which lies outside both lines, find that intersection point in cartesian space and add it to the collection of control points */
  		if(((t[0]<0)&&(t[1]>1))||((t[0]>1)&&(t[1]<0)))
  		{
  			float x,y;	
  			if(t[0]<t[1])
  			{
  				if(group_of_lines[i][j]>group_of_lines[i][j+1])
  				{
  					x = ((input_lines.lines[group_of_lines[i][j+1]][0])*(1-t[1]))+((input_lines.lines[group_of_lines[i][j+1]][2])*t[1]);
  					y = ((input_lines.lines[group_of_lines[i][j+1]][1])*(1-t[1]))+((input_lines.lines[group_of_lines[i][j+1]][3])*t[1]);
  				}
  				else
  				{
  					x = ((input_lines.lines[group_of_lines[i][j]][0])*(1-t[1]))+((input_lines.lines[group_of_lines[i][j]][2])*t[1]);
  					y = ((input_lines.lines[group_of_lines[i][j]][1])*(1-t[1]))+((input_lines.lines[group_of_lines[i][j]][3])*t[1]);
  				}
  			}
  			else
  			{
  				if(group_of_lines[i][j]>group_of_lines[i][j+1])
  				{
  					x = ((input_lines.lines[group_of_lines[i][j]][0])*(1-t[0]))+((input_lines.lines[group_of_lines[i][j]][2])*t[0]);
  					y = ((input_lines.lines[group_of_lines[i][j]][1])*(1-t[0]))+((input_lines.lines[group_of_lines[i][j]][3])*t[0]);
  				}
  				else
  				{
  					x = ((input_lines.lines[group_of_lines[i][j+1]][0])*(1-t[0]))+((input_lines.lines[group_of_lines[i][j+1]][2])*t[0]);
  					y = ((input_lines.lines[group_of_lines[i][j+1]][1])*(1-t[0]))+((input_lines.lines[group_of_lines[i][j+1]][3])*t[0]);
  				}
  			}	 
  			temp_control_points.push_back(cv::Vec2f(x,y));
  		}
  	}	
  }
  curves.control_points.push_back(temp_control_points);	// Store the control points in the data-structure for curves
 }
 
 /* At this point, the control points for each curve has been extracted and stored in the data-structure "curves" */
 /* This loop estimates the curve as a set of sampled points based on the curve resolution specified and stores the sampled points in the "curves" data-structure. The drawing points are then used by the calling function to draw the piece-wise linear bezier curve on the output frames */
 for(int i=0;i<curves.control_points.size();i++)
 {
 	temp_drawing_points.clear();		// Clear the data-structure used to store the set of drawing points
 	std::vector<cv::Vec2f> drawing_point;	// A temporary variable to store a set of drawing points
 	//printf("\n Lane %d:",i);
 	sort_points(&(curves.control_points[i])); // Sort the control points of a curve according to their position on the image from lower 							  portion of the image to higher (high y co-ordinate to low y co-ordinate)
	
	/* This loop samples the curve at specified sampling rate and collects all drawing points(sampled points)*/
 	for(float j=0; j<=1;j+=CURVE_RESOLUTION)
 	{
 		drawing_point=evaluate_curve(curves.control_points[i],j); // Evaluate the curve at parameter j(varies from 0 to 1)
 		temp_drawing_points.push_back(cv::Vec2f(drawing_point[0][0],drawing_point[0][1])); //Add the sampled point to the 													   collection of temporary drawing points
 	}
 	curves.drawing_points.push_back(temp_drawing_points);// Store all the collected drawing points for each curve to the data-structure
 } 
 
 /* At this point, all the drawing points for all the curves are extracted and stored in the data-structure "curves" */
 return curves;			// Return the data-structure to the calling function
}
/* ------------------------------------------------------------------------------------------------------------ */ 


/* This function receives a set of 2-D points and sorts them in descending order of their y co-ordinate value */
/* ------------------------------------------------------------------------------------------------------------ */ 
void sort_points(std::vector<cv::Vec2f> *points)
{
  cv::Vec2f swap;
  for (int i=0; i<((*points).size()-1); i++)
  {
    for (int j=0; j<((*points).size()-i-1); j++)
    {
      if ((*points)[i][1] < (*points)[i+1][1]) 
      {
        swap       = (*points)[i];
        (*points)[i]  = (*points)[i+1];
        (*points)[i+1] = swap;
      }
    }
  }
}
/* ------------------------------------------------------------------------------------------------------------ */ 


/* This function generates the bezier curve using the recursive De Casteljau's algorithm for a given set of control points and samples the curve at the specified time_index (from 0 to 1) to return a drawing point */
/* ------------------------------------------------------------------------------------------------------------ */ 
std::vector<cv::Vec2f> evaluate_curve(std::vector<cv::Vec2f> control_points, float time_index)
{
	std::vector<cv::Vec2f> next_control_points;	// A variable to store the control points for next recursion 
	float t1=(1-time_index), t2=time_index;	// Get the time_index at which the curve has to be sampled in terms of t1 and t2 of the 						De Casteljau's algorithm
	if(control_points.size()==1)		// If the received set of points is just one, then the algorithm has reached its last 							recrusion. Return the same point
	{
		return control_points;
	}
	else					// If not, find the next set of control points which are in between two 						neighboring control points at a distance defined by time index
	{
		for(int i=0;i<(control_points.size()-1);i++)
		{
			next_control_points.push_back(cv::Vec2f(((control_points[i][0]*t1)+(control_points[i+1][0]*t2)), ((control_points[i][1]*t1)+(control_points[i+1][1]*t2))));
		}
	}
	next_control_points=evaluate_curve(next_control_points,time_index); // Recursively calculate a single drawing point
	return next_control_points;	// Return the set of control points for next recursion. The initial calling function receives only 						one drawing point evaluated at the specified time index
}
/* ------------------------------------------------------------------------------------------------------------ */ 


/* This function simply traverses the connectivity tree recursively and prints out the tree structure. Only used during development stage*/
/* ------------------------------------------------------------------------------------------------------------ */ 
int traverse_tree(struct node* traverser)
{
	for(int i=0;i<traverser->connections;i++)
	{
		//printf("\n");
		traverse_tree(traverser->connectors[i]);
		//printf("\t %d <--", traverse_tree(traverser->connectors[i]));
	}
	return traverser->line_number;
}
/* ------------------------------------------------------------------------------------------------------------ */ 


/* This function extracts the minimum-cost path from the traveser node to any reachable leaf node through recursion */
/* ------------------------------------------------------------------------------------------------------------ */ 
void extract_lines(struct node* traverser, std::vector<int> *container)
{
	if(traverser->min_path_to_child !=NULL)	// If there is a child which lies on minimum-cost path
	{
		// If that child's minimum-cost parent is traverser, then the parent and child pair belongs to a minimum-cost path
		if(traverser->min_path_to_child->min_path_from_parent->line_number == traverser->line_number)
		{
			(*container).push_back(traverser->min_path_to_child->line_number);
			extract_lines(traverser->min_path_to_child, container); // Extract lines for the child node recursively
		}
	}	
}
/* ------------------------------------------------------------------------------------------------------------ */ 


/* This function calculates the global cost of connectivity to each children of the traverser and returns the minimum-cost to connect to any of its children. This implementation is based on dynamic programming */
/* ------------------------------------------------------------------------------------------------------------ */ 
double find_min_cost_among_children(struct node* traverser)
{
 double min_cost=0, temp_min_cost;		// Variables to store minimum cost and temporary variable to store a possible minimum cost
 traverser->min_path_to_child=NULL;		// Assume there is no child with minimum cost
	for(int i=0;i<traverser->connections;i++) // For all children of the traverser
	{
		if(i==0)	// For first child, assume this child is the min-cost child and recursively find out its global cost
		{
			min_cost=find_min_cost_among_children(traverser->connectors[i])-1+traverser->cost_to_connect[i];
			traverser->min_path_to_child=traverser->connectors[i];
		}
		else	// for subsequent children, recursively find out global cost of connectivity and check if its the minimum cost 
		{
			temp_min_cost=find_min_cost_among_children(traverser->connectors[i])-1+traverser->cost_to_connect[i];
			if(temp_min_cost<min_cost)	// If this is the minimum cost so far, then remember this cost and the child 
			{
				min_cost=temp_min_cost;
				traverser->min_path_to_child=traverser->connectors[i];
			}
		}
	}
	/* At this point, "min_cost" will have the minimum possible global cost of connectivity from a parent to a leaf node  */
	traverser->min_cost_to_child=min_cost;	// Store that cost within the tree data-structure
	return min_cost; // Return that minimum cost
}
/* ------------------------------------------------------------------------------------------------------------ */ 


/* This function calculates the global cost of connectivity from each parent of the traverser and returns the minimum-cost provided among all of its parents. This implementation is based on dynamic programming */
/* ------------------------------------------------------------------------------------------------------------ */ 
double find_min_cost_among_parents(struct node* traverser)
{
 double min_cost=0, temp_min_cost;		// Variables to store minimum cost and temporary variable to store a possible minimum cost
 traverser->min_path_from_parent=NULL;		// Assume there is no parent which provides minimum cost
	for(int i=0;i<traverser->parents;i++)	// For all parents of the traverser
	{
		int k;
		if(i==0)	// For first parent, assume this parent is the min-cost parent and recursively find out its global cost
		{
			min_cost=find_min_cost_among_parents(traverser->parent_connectors[i]);
			for(k=0;k<traverser->parent_connectors[i]->connections;k++)
			{
				if(traverser->parent_connectors[i]->connectors[k]->line_number == traverser->line_number)
				{
					break;
				}
			}
			min_cost=min_cost-1+(traverser->parent_connectors[i]->cost_to_connect[k]);
			traverser->min_path_from_parent=traverser->parent_connectors[i];
		}
		else	// for subsequent parents, recursively find out global cost of connectivity and check if its the minimum cost 
		{
			temp_min_cost=find_min_cost_among_parents(traverser->parent_connectors[i]);
			for(k=0;k<traverser->parent_connectors[i]->connections;k++)
			{
				if(traverser->parent_connectors[i]->connectors[k]->line_number == traverser->line_number)
				{
					break;
				}
			}
			temp_min_cost=temp_min_cost-1+(traverser->parent_connectors[i]->cost_to_connect[k]);
			if(temp_min_cost<min_cost)	// If this is the minimum cost so far, then remember this cost and the parent 
			{
				min_cost=temp_min_cost;
				traverser->min_path_from_parent=traverser->parent_connectors[i];
			}
		}
	}
	/* At this point, "min_cost" will have the minimum possible global cost of connectivity to a child from one of its parents */
	traverser->min_cost_from_parent=min_cost;	// Store that cost within the tree data-structure
	return min_cost;	// Return that minimum cost
}
/* ------------------------------------------------------------------------------------------------------------ */ 
