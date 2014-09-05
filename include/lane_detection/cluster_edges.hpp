/* Copyright (C) Keerthi Raj Nagaraja - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Keerthi Raj Nagaraja <nagaraj1@purdue.edu>, July 2014
 */
 
#ifndef CLUSTER_EDGES_H
#define CLUSTER_EDGES_H

#include <vector>

/* Function declarations for the functions in cluster_edges.cpp source file*/
std::vector<struct clustered_lines> cluster(struct detected_lines input_lines);
struct fitted_lines weighted_linear_fit(std::vector<struct clustered_lines>, int);

#endif
