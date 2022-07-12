/*
 * Flight.cpp
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


#include "std_msgs/String.h"

#include "Flight.h"

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#include <GL/gl.h>
#endif

#include "Common.h"

#include <iostream>
#include <string>
#include <math.h>

Flight::~Flight() {
	// TODO Auto-generated destructor stub
}

Flight::Flight(std::string _id, std::string _type, Position _pos, float _bearing, float _inclination, float _speed, int _terminal, int _runaway_arrival, int _runaway_departure, int _highspeed, float _generation_time, std::string _airline, int _capacity, int _occupation){
	id = _id;
	type = _type;
	pos = _pos;
	bearing_ = _bearing;
	inclination_ = _inclination;
	terminal = _terminal;
	runaway_arrival = _runaway_arrival;
	runaway_departure = _runaway_departure;
	highspeed = _highspeed;
	generation_time = _generation_time;
	airline = _airline;
	capacity = _capacity;
	occupation = _occupation;

	setSpeed(_speed);	// Through set in order to limit speeds

	route.clear();

	if (type == "A320"){
		points_ = INIT_FLIGHT_POINTS;
	}else if(type == "A380"){
		points_ = INIT_FLIGHT_POINTS*2;
	}
	w_speed = 0.0f;
}

void
Flight::update(float delta_t)
{
	Position CPpos;
	bool ground = false;
	float diff_bearing, diff_inclination, goal_speed, diff_speed, acc;
	if(routed())
	{
		CPpos = route.front().pos;
		if(pos.get_z() > 0.05){
			float goal_bearing, new_w, goal_inc, t_bearing;

			if(CPpos.get_z() <= MAINTAIN_ALT){ //Maintain altitude
				float current_alt = (this->getPosition()).get_z();
				CPpos.set_z(current_alt);
				route.front().pos.set_z(current_alt);
			}
			pos.angles(CPpos, goal_bearing, goal_inc);	
			goal_bearing = normalizePi(goal_bearing + M_PI);
			diff_bearing = normalizePi(goal_bearing - bearing_);
			new_w = diff_bearing;

			bearing_ = bearing_ + new_w*delta_t;

			inclination_ = goal_inc;

			//Maintain speed while turning
			if(fabs(new_w) < 0.6*0.9){
				goal_speed = route.front().speed;
				if(route.front().speed<0){
			 		goal_speed = speed;
				}
			}else{
				goal_speed = speed;
			}
			acc = (goal_speed - speed);
			if(fabs(acc)>MAX_ACELERATION) acc = (acc/fabs(acc))*MAX_ACELERATION;

		}else{
			ground = true;
			float goal_bearing, goal_inc;
			pos.angles(CPpos, goal_bearing, goal_inc);
			inclination_ = goal_inc;
			goal_bearing = normalizePi(goal_bearing + M_PI);
			bearing_ = goal_bearing;
			goal_speed = route.front().speed;
			acc = (goal_speed - speed);
			if(fabs(acc)>MAX_ACELERATION) acc = (acc/fabs(acc))*MAX_ACELERATION;
		}
	}else{
		inclination_ = 0.0;
	}

	speed = speed + acc*delta_t;
	float trans = speed*0.1;
	
	float factor, rest;
	factor= 0;
	if(ground){
		float t_distance = trunc(pos.distanceXY(CPpos)/trans);
		rest = pos.distance(CPpos) - t_distance * trans;
		if(t_distance != 0){
			factor = rest / t_distance;
		}
		if(trunc(pos.distance(CPpos)) <= (trunc(trans + factor) + 1)){ 
			route.pop_front();
			this->setNWaypArrive(this->getNWaypArrive()-1);
		}
	}else{
		if(pos.distance(CPpos) <= trans + 1){ 
			route.pop_front();
			this->setNWaypArrive(this->getNWaypArrive()-1);
		}
	}

	pos.set_x(pos.get_x() + ((factor + trans) * cos(bearing_) * cos(inclination_)));
	pos.set_y(pos.get_y() + ((factor + trans) * sin(bearing_) * cos(inclination_)));
	
	pos.set_z(round((pos.get_z() + trans * sin(inclination_)) * 100) / 100);
	if(pos.get_z() < 0.05){
		pos.set_z(0);
	}
}

float Flight::checkSpeedLimits(float tgt_speed){
	return (tgt_speed > CRASH_SPEED_MAX*DIMENSION_FACTOR ? CRASH_SPEED_MAX*DIMENSION_FACTOR : tgt_speed);
}

int Flight::at_crash_point(){
	if(route.front().pos.get_x() == 1170 && route.front().pos.get_y() == -4050){
		return 1;  //CRITICAL POINT 1
	}else if(route.front().pos.get_x() == 1450 && route.front().pos.get_y() == -4800){
		return 2;  //CRITICAL POINT 2
	}else if(route.front().pos.get_x() == 2340 && route.front().pos.get_y() == -3650){
		return 3;  //CRITICAL POINT 3
	}else if(route.front().pos.get_x() == 1430 && route.front().pos.get_y() == -5030){
		return 4;  //CRITICAL POINT 4
	}else if(route.front().pos.get_x() == 2120 && route.front().pos.get_y() == -3580){
		return 5;  //CRITICAL POINT 5
	}else if(route.front().pos.get_x() == 960 && route.front().pos.get_y() == -3200){
		return 6;  //CRITICAL POINT 6
	}else if(route.front().pos.get_x() == 1650 && route.front().pos.get_y() == -2900){
		return 7;  //CRITICAL POINT 7
	}else if(route.front().pos.get_x() == 2670 && route.front().pos.get_y() == -3120){
		return 8;  //CRITICAL POINT 8
	}else if(route.front().pos.get_x() == 2910 && route.front().pos.get_y() == -2350){
		return 9;  //CRITICAL POINT 9
	}else if(route.front().pos.get_x() == 880 && route.front().pos.get_y() == -3450){
		return 10;  //CRITICAL POINT 10
	}else{
		return 0;
	}
}

bool 
Flight::atTerminal(){
	Position Terminal1, Terminal2, Terminal3;
	//T4S
	Terminal1.set_x(2200);
	Terminal1.set_y(-3300);
	Terminal1.set_z(0);
	//T4
	Terminal2.set_x(200);
	Terminal2.set_y(-3230);
	Terminal2.set_z(0);
	//T123
	Terminal3.set_x(1660);
	Terminal3.set_y(-5900);
	Terminal3.set_z(0);

	if(pos.distance(Terminal1) < 1.1){
		return true;
	}else if(pos.distance(Terminal2) < 1.1){
		return true;
	}else if(pos.distance(Terminal3) < 1.1){
		return true;
	}else{
		return false;
	}
}

bool
Flight::at_P0_Runaway(){
	Position L36_d, R36_d;
	R36_d.set_x(3150);
	R36_d.set_y(-2425);
	R36_d.set_z(0);
	L36_d.set_x(1800);
	L36_d.set_y(-2400);
	L36_d.set_z(0);

	if(route.front().pos.get_x() == R36_d.get_x() && route.front().pos.get_y() == R36_d.get_y()){
		return true;
	}else if(route.front().pos.get_x() == L36_d.get_x() && route.front().pos.get_y() == L36_d.get_y()){
		return true;
	}else{
		return false;
	}
}

bool
Flight::at_a_Runaway(){
	Position R32_1, L32_1, R32_2, L32_2;
	R32_1.set_x(3450);
	R32_1.set_y(-4080);
	R32_1.set_z(0);
	R32_2.set_x(4000);
	R32_2.set_y(-5600);
	R32_2.set_z(0);
	
	L32_1.set_x(1720);
	L32_1.set_y(-5000);
	L32_1.set_z(0);
	L32_2.set_x(2175);
	L32_2.set_y(-6200);
	L32_2.set_z(0);

	if((route.front().pos.get_x() == R32_1.get_x() && route.front().pos.get_y() == R32_1.get_y()) || (route.front().pos.get_x() == R32_2.get_x() && route.front().pos.get_y() == R32_2.get_y())){
		return true;
	}else if((route.front().pos.get_x() == L32_1.get_x() && route.front().pos.get_y() == L32_1.get_y()) || (route.front().pos.get_x() == L32_2.get_x() && route.front().pos.get_y() == L32_2.get_y())){
		return true;
	}else{
		return false;
	}
}

bool
Flight::at_d_Runaway(){
	Position L36_d, R36_d;
	R36_d.set_x(2750);
	R36_d.set_y(-500);
	R36_d.set_z(0);
	L36_d.set_x(2400);
	L36_d.set_y(-450);
	L36_d.set_z(0);

	if(route.front().pos.get_x() == R36_d.get_x() && route.front().pos.get_y() == R36_d.get_y()){
		return true;
	}else if(route.front().pos.get_x() == L36_d.get_x() && route.front().pos.get_y() == L36_d.get_y()){
		return true;
	}else{
		return false;
	}
}

bool 
Flight::at_waiting_zone(){
	Position R32_a, L32_a;
	R32_a.set_x(5200);
	R32_a.set_y(-9000);
	R32_a.set_z(0);
	L32_a.set_x(3200);
	L32_a.set_y(-9000);
	L32_a.set_z(0);
	
	if(route.front().pos.get_x() == R32_a.get_x() && route.front().pos.get_y() == R32_a.get_y()){
		return true;
	}else if(route.front().pos.get_x() == L32_a.get_x() && route.front().pos.get_y() == L32_a.get_y()){
		return true;
	}else{
		return false;
	}
}

void 
Flight::waiting_circ(){
	std::list<Route>::iterator it;
	it = this -> getRoute() -> begin();
	Route R0, R1, R2, R3, R4, R5;
	Position P0, P1, P2, P3, P4, P5;
	P0.set_x(pos.get_x() + 500);
	P0.set_y(pos.get_y() + 250);
	P0.set_z(pos.get_z());
	R0.pos = P0;
	R0.waiting = true;
	R0.speed = this -> getSpeed();
	this -> getRoute() -> insert(it, R0);
	this -> setNWaypArrive(this -> getNWaypArrive() + 1);
	P1.set_x(pos.get_x() + 1000);
	P1.set_y(pos.get_y());
	P1.set_z(pos.get_z());
	R1.pos = P1;
	R1.waiting = true;
	R1.speed = this -> getSpeed();
	this -> getRoute() -> insert(it, R1);
	this -> setNWaypArrive(this -> getNWaypArrive() + 1);
	P2.set_x(pos.get_x() + 1000);
	P2.set_y(pos.get_y() - 1500);
	P2.set_z(pos.get_z());
	R2.pos = P2;
	R2.waiting = true;
	R2.speed = this -> getSpeed();
	this -> getRoute() -> insert(it, R2);
	this -> setNWaypArrive(this -> getNWaypArrive() + 1);
	P3.set_x(pos.get_x() + 500);
	P3.set_y(pos.get_y() - 1750);
	P3.set_z(pos.get_z());
	R3.pos = P3;
	R3.waiting = true;
	R3.speed = this -> getSpeed();
	this -> getRoute() -> insert(it, R3);
	this -> setNWaypArrive(this -> getNWaypArrive() + 1);
	P4.set_x(pos.get_x());
	P4.set_y(pos.get_y() - 1500);
	P4.set_z(pos.get_z());
	R4.pos = P4;
	R4.waiting = true;
	R4.speed = this -> getSpeed();
	this -> getRoute() -> insert(it, R4);
	this -> setNWaypArrive(this -> getNWaypArrive() + 1);
	P5.set_x(pos.get_x());
	P5.set_y(pos.get_y());
	P5.set_z(pos.get_z());
	R5.pos = P5;
	R5.waiting = true;
	R5.speed = this -> getSpeed();
	this -> getRoute() -> insert(it, R5);
	this -> setNWaypArrive(this -> getNWaypArrive() + 1);
}


