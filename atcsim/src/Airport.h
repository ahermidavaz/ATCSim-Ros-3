/*
 * Airport.h
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


#ifndef AIRPORT_H
#define AIRPORT_H

#include <memory>
#include <vector>
#include <string>

#include "Flight.cpp"

typedef struct {
	std::string airline;
	int pax;
	std::string type;
	std::string id;
} Results;

class Airport {
public:
	Airport(): n_() {

   	obj_ts_ = ros::Time::now();
	obj_ts_2 = ros::Time::now();
	arrayflight_info_pub_ = n_.advertise<atcsim_msgs::ArrayFlight>("arrayflight_info", 1000);
	p_arival_info_pub = n_.advertise<atcsim_msgs::Controller>("p_arival_info", 1000);
	flightMarkers_pub_ = n_.advertise<visualization_msgs::MarkerArray>( "visualization_flight_marker", 0 );
	airportMarker_pub_ = n_.advertise<visualization_msgs::MarkerArray>( "visualization_airport_marker", 0 );

	mult_speed_ = 1;
	local_time_= 0;
	num_mark_ = 0;
	departures = 0;
	arrivals = 0;
	pax_a = 0;
	pax_d = 0;
	T123 = 0;
	T4 = 0;
	T4S = 0;
	_32L = 0;
	_32R = 0;
	_36L = 0;
	_36R = 0;

  };
  	void doWork();
  	void handling();
  	void check_waiting();
	void checkCollisions();
	void checkCrashes();
	void checkLandings();
	void set_mult_speed(float mult){
		if (mult < 0.5){ mult = 0.5;}
		if (mult > 5)  { mult = 5;}
		mult_speed_ = mult;
	}

	bool command(atcsim_msgs::CommanderService::Request& req, atcsim_msgs::CommanderService::Response& res);

	bool sendInfo(atcsim_msgs::InfoService::Request& req,
	       	atcsim_msgs::InfoService::Response& res);

	visualization_msgs::Marker add_marker(std::string id, std::string type, int count, int mir);
	visualization_msgs::Marker add_marker_sphere_flight(std::string id, std::string type, int count);
	visualization_msgs::Marker drawWp(boost::shared_ptr<Flight> flight, int count);


	void drawInfo();
	void drawHelpers(int num);
	
	void V_Calc();
	
	void check_departures();
	void results();
	
	int get_departures(){return departures;};
	int get_T123(){return T123;};
	int get_T4(){return T4;};
	int get_T4S(){return T4S;};
	

private:

	std::list<boost::shared_ptr<Flight>> flights_;

	ros::Time obj_ts_;
	ros::Time obj_ts_2;
	int num_mark_;

  	ros::NodeHandle n_;
	ros::Publisher arrayflight_info_pub_;
	ros::Publisher p_arival_info_pub;
	ros::Publisher airportMarker_pub_;
	ros::Publisher flightMarkers_pub_;
	ros::Publisher arrayflight_;

	tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped flight_obj_msg;

	float AirportPoints_;
	float mult_speed_;
	float local_time_;
	int departures, arrivals;
	int pax_a, pax_d;
	int T123, T4, T4S; 
	int _32L, _32R, _36L, _36R;
	std::list<Results> arrivals_list, departures_list;

};

#endif
