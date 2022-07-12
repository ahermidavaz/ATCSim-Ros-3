/*
 * opt.cpp
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
#include "atcsim_msgs/HandlingService.h"
#include "atcsim_msgs/CommanderService.h"
#include "atcsim_msgs/Controller.h"
#include <math.h>

#include <vector>
#include <string>
#include <list>

#include <sys/time.h>
#include "Common.h"
#include "Flight.cpp"

class Control{
public:
    Control();
    virtual ~Control(){};
    void fill(const atcsim_msgs::Controller msg);
    bool opti(atcsim_msgs::HandlingService::Request &req, atcsim_msgs::HandlingService::Response &res);
    bool get_L_busy(){return L_busy;};
    bool get_R_busy(){return R_busy;};
    void set_L_busy(bool L_busy_){L_busy = L_busy_;};
    void set_R_busy(bool R_busy_){R_busy = R_busy_;};

private:
    bool L_busy;
    bool R_busy;
};

Control::Control(){
    this -> L_busy = false;
    this -> R_busy = false;
}

void
Control::fill(const atcsim_msgs::Controller msg){
    if(msg.L_waiting == true){
        this -> set_L_busy(true);
    }else{
        this -> set_L_busy(false);
    }
    
    if(msg.R_waiting == true){
        this -> set_R_busy(true);
    }else{
        this -> set_R_busy(false);
    }
}

bool 
Control::opti(atcsim_msgs::HandlingService::Request &req, atcsim_msgs::HandlingService::Response &res){

    int rann = 1 + rand()%(3);

    if(req.airline == "RYANAIR" || req.airline == "AIR FRANCE"){
        res.terminal = 3; //T123
    }else if(req.airline == "VUELING" || req.airline == "BRITISH AIRWAYS"){
        res.terminal = 1; //T4S
    }else if(req.airline == "IBERIA"){
        res.terminal = 2; //T4
    }else{
        res.terminal = rann;
    }

    bool busy_airport = false;
    
    //100% AIRPORT
    if(req.posi_x > 5000){
        if(this -> get_R_busy() != true){
            res.a_runaway = 4;
        }else if(this -> get_L_busy() != true){
            res.a_runaway = 3;
        }else{
            busy_airport = true;
        }
    }else{
        if(this -> get_L_busy() != true){
            res.a_runaway = 3;
        }else if(this -> get_R_busy() != true){
            res.a_runaway = 4;
        }else{
            busy_airport = true;
        }
    }

    int i = rand()%2;
    /*if(i == 0){
        res.d_runaway = 1;
    }else{
        res.d_runaway = 2;
    }*/

    //50%AIRPORT
    //res.a_runaway = 3;
    res.d_runaway = 1;

    if(busy_airport != true){
        res.generate = true;
    }else{
        res.generate = false;
    }

    return true;
}

int main(int argc, char **argv)
{
    std::cerr<< std::endl<<" ---------- OPT IS ON ----------"<< std::endl<<std::endl;
    Control control;
    ros::init(argc, argv, "opt");
    ros::NodeHandle n;
    ros::ServiceServer s = n.advertiseService("handling_service", &Control::opti, &control);
    ros::Subscriber sub = n.subscribe("p_arival_info", 1000, &Control::fill, &control);
    ros::spin();
    return 0;
}

    