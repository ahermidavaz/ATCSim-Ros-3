/*
 * Flight.h
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


#ifndef FLIGHT_H_
#define FLIGHT_H_
#include <string>
#include "Position.h"
#include "Position.cpp"
#include "Common.h"
#include <list>

#include <visualization_msgs/Marker.h>

typedef struct {
	Position pos;
	float speed;
	bool waiting = false;
} Route;

class Flight {
public:
	Flight(std::string _id, std::string _type, Position _pos, float _bearing, float _inclination, float _speed, int _terminal, int _runaway_arrival, int _runaway_departure, int _highspeed, float _generatio_time, std::string _airline, int _capacity, int _occupation);
	virtual ~Flight();

	void update(float delta_t);

	std::list<Route> *getRoute() {return &route;};
	bool routed() { return !route.empty();};
	Position getPosition() { return pos;};
	float getInclination() { return inclination_;};
	float getBearing() { return bearing_;};
	float getSpeed() { return speed;};
	
	//void setSpeed(float tgt_speed) {speed = checkSpeedLimits(tgt_speed);}
	
	void setSpeed(float tgt_speed) {speed = tgt_speed;}
	
	float getPoints() {return points_;};
	std::string getId() {return id;};
	std::string getType(){return type;};
	std::string getAirline(){return airline;};
	
	int getTerminal(){return terminal;};
	int getRunaway_arrival(){return runaway_arrival;};
	int getRunaway_departure(){return runaway_departure;};
	int getHighspeed(){return highspeed;};
	
	int at_crash_point();
	bool atTerminal();
	bool at_P0_Runaway();
	bool at_a_Runaway();
	bool at_d_Runaway();
	bool at_waiting_zone();
	void waiting_circ();
	
	void setArrival_Time(ros::Time Time) {arrival_time = Time;};
	void setDeparture_Time(ros::Time Time) {departure_time = Time;};
	
	ros::Time getArrival_Time() {return arrival_time;};
	ros::Time getDeparture_Time() {return departure_time;};
	
	int getHandling() {return handling;};
	void setHandling() {handling++;};
	
	void setNWaypArrive(int n) {nwayparrive = n;};
	int getNWaypArrive() {return nwayparrive;};
	
	void setWaiting(bool Waiting){waiting = Waiting;};
	bool getWaiting(){return waiting;};
	
	float getGeneration_time(){return generation_time;};
	void setGeneration_time(){generation_time = generation_time + 40;};
	
	int getCapacity(){return capacity;};
	
	int getOccupation(){return occupation;};

private:
	std::string id;
	std::string type;
	std::string airline;
	Position pos, last_pos;
	float bearing_, inclination_;
	float speed, w_speed;
	std::list<Route> route;
	float points_;
	float checkSpeedLimits(float tgt_speed);
	int terminal, highspeed, runaway_arrival, runaway_departure, capacity, occupation; 
	int handling = 0; //0 BEFORE LANDING, 1 HANDLING, 2 AFTER LANDING  //CORREGIR
	bool waiting;
	int nwayparrive;
	ros::Time arrival_time;
	ros::Time departure_time;
	float generation_time;
	
};


#endif /* FLIGHT_H_ */
