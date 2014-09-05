/* Copyright (C) Keerthi Raj Nagaraja - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Keerthi Raj Nagaraja <nagaraj1@purdue.edu>, July 2014
 */
 
#ifndef BEZIER_CURVES_H
#define BEZIER_CURVES_H

#include <vector>

/* Function declarations for the functions in bezier_curve.cpp source file*/
struct bezier_curves generate_curves(struct fitted_lines);
int traverse_tree(struct node*);
double find_min_cost_among_children(struct node*);
double find_min_cost_among_parents(struct node*);
void extract_lines(struct node*, std::vector<int>*);
std::vector<cv::Vec2f> evaluate_curve(std::vector<cv::Vec2f>, float);
void sort_points(std::vector<cv::Vec2f>*);

#endif
