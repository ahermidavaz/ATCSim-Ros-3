/*
 * Barajas.h
 *
 *  Created on: 04/2022
 *
 *
 *  Copyright 2022 Alberto Hermida
 *
 *  This file is part of ATCSimROS_2.
 *
 *  ATCSimROS_2 is free software based on the ATCsim project , created for a
 *	final project of the Aerospace Engineering at URJC (Madrid).
 *	Author: Alberto Hermida Vazquez (ahermidavazquez@gmail.com)
 *	Tutor: Jose Miguel Guerrero
 *  Author of ATCsim project: Francisco Martin
 * 
 *  ATCSim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 */


#ifndef BARAJAS_H
#define BARAJAS_H


#include "ros/ros.h"
#include <iostream>
#include <list>
#include <memory>
#include <vector>
#include <string>

class Airport_shape {
public:
	Airport_shape(): n_() {

		airport_terminal_pub = n_.advertise<visualization_msgs::MarkerArray>("visualization_terminal_marker", 0);
		generate_axis_pub = n_.advertise<visualization_msgs::MarkerArray>("AXIS", 0);
  };
  	void generate_airport_terminal();
  	void generate_axis();


private:
	ros::NodeHandle n_;
	ros::Publisher airport_terminal_pub;
	ros::Publisher generate_axis_pub;
};

#endif
