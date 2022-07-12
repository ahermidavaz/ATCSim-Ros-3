/*
 * atcsim.cpp
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

#include "ros/ros.h"
#include "Airport.cpp"
#include "Barajas.cpp"
#include "Axis.cpp"
#include <sys/time.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "atcsim_node");
  ros::NodeHandle n;

  ros::Rate loop_rate(10);

  Airport_shape terminal;
  Airport airport;

  ros::ServiceServer ss = n.advertiseService("commander_service", &Airport::command, &airport);
 
  std::cerr<< std::endl<<" ---------- ATCSIM IS ON ----------"<< std::endl<<std::endl;

  while (ros::ok())
  {
    airport.doWork();
    airport.handling();
    airport.check_waiting();
    airport.check_departures();
    airport.results();
    terminal.generate_airport_terminal();
    terminal.generate_axis();
    ros::spinOnce(); 
    loop_rate.sleep();
  }
  return 0;
}
