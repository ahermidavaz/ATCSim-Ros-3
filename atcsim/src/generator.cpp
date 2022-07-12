/*
 * generator.cpp
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
#include "atcsim_msgs/CommanderService.h"
#include "atcsim_msgs/HandlingService.h"

#include <sys/time.h>
#include <math.h>
#include "Common.h"
#include "Position.cpp"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "generator");
  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<atcsim_msgs::CommanderService>("commander_service");
  ros::ServiceClient client2 = n.serviceClient<atcsim_msgs::HandlingService>("handling_service");
  atcsim_msgs::CommanderService srv;
  atcsim_msgs::HandlingService srv2;

  ros::Time obj_ts = ros::Time::now();
  ros::Time s_time = ros::Time::now();
  srv2.request.starting_time = ros::Time::now().toSec();
  
  float time_generation = 20;
  int num = 0;
  int type_count = 0;
  int capacity, occupation;
  std::string type, airline;

  std::cerr<< std::endl<<" ------- FLIGHT GENERATION IS ON -------"<< std::endl<<std::endl;

  while (ros::ok() && type_count == 0){
    if((ros::Time::now() - obj_ts).toSec() > time_generation){
      obj_ts = ros::Time::now(); 
      srv2.request.time = (ros::Time::now()-s_time).toSec();

      int rann = rand()%(100);
      int rann2;
      char id[6];
 
      if(rann <= 37){
        airline = "IBERIA";
        sprintf(id, "IB%4.4d", num);
        rann2 = rand()%(2);
        if(rann2 == 0){
          type = "A320";
        }else if(rann2 == 1){
          type = "A330";
        }
      }else if(rann <= 38 && rann > 37){
        airline = "AIR FRANCE";
        sprintf(id, "AF%4.4d", num);
        rann2 = rand()%(2);
        if(rann2 == 0){
          type = "A320";
        }else if(rann2 == 1){
          type = "A330";
        }
      }else if(rann <= 40 && rann > 38){
        airline = "VUELING";
        sprintf(id, "VU%4.4d", num);
        type = "A320";
      }else if(rann <= 50 && rann > 40){
        airline = "RYANAIR";
        sprintf(id, "RY%4.4d", num);
        type = "B737";
      }else if(rann <= 51 && rann > 50){
        airline = "BRITISH AIRWAYS";
        sprintf(id, "BA%4.4d", num);
        type = "A320";
      }else{
        airline = "ANOTHERS";
        sprintf(id, "XX%4.4d", num);
        type = "A320";
      }

      if(type == "A320"){
        capacity = 180;
      }else if(type == "B737"){
        capacity = 189;
      }else if(type == "A330"){
        capacity = 268;
      }else if(type == "B787"){
        capacity = 290;
      }else{
        capacity = 348;
      }

      int occupation = 70 + rand()%(21);

      srv2.request.airline = airline;
      srv2.request.type = type;

    //INTIAL POSITION

      float x, y, z, bear, inc;
      x = -3000 + rand() % (16001);
      y = -sqrt(218750000 - x*x + 5000*x) - 3000;
    	z = (FLIGHT_HEIGHT + (float)(rand() % 1001 ));
 
    	Position ipos(x, y, z);
    	Position pos0(2500.0, -3000.0, 0.0);

    	pos0.angles(ipos, bear, inc);

      srv.request.code = 1;
      srv.request.id = id;
      srv.request.type = type;
      srv.request.airline = airline;
      srv.request.capacity = capacity;
      srv.request.occupation = occupation;
      srv.request.posx = x;
      srv.request.posy = y;
      srv.request.posz = z;
      int v = 350 + rand()%(51);
      srv.request.speed = v*DIMENSION_FACTOR;
      srv.request.bearing = bear; 
      srv.request.inclination = inc;
      srv2.request.posi_x = x;
      srv2.request.posi_y = y;
      srv.request.generation_time = (ros::Time::now()-s_time).toSec();

      if(client2.call(srv2)){
        if(!(srv2.response.achieved)){
           ROS_INFO_STREAM("FAIL");
        }
      }else{
        ROS_ERROR("Failed to call service");
        return 1;
      }

      int highspeed = 1+rand()%(3-1);
      srv.request.terminal = srv2.response.terminal;
      srv.request.a_runaway = srv2.response.a_runaway;
      srv.request.d_runaway = srv2.response.d_runaway;
      srv.request.highspeed = highspeed;

      //ARRIVALS

      atcsim_msgs::Waypoint wp_req;
      srv.request.wps.clear();
      if(srv2.response.a_runaway == 3){  //ARRIVALS 32L
        wp_req.x = 3200; //WAITING ZONE
        wp_req.y = -9000;                                
        wp_req.z = 300;
        wp_req.speed = v*DIMENSION_FACTOR;
        wp_req.llegada = false;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 2700;  //INICIO PISTA 32L
        wp_req.y = -7650;
        wp_req.z = 100;
        wp_req.speed = 250*DIMENSION_FACTOR;
        wp_req.llegada = false;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 1950;  //GROUND
        wp_req.y = -5600;
        wp_req.z = 0;
        wp_req.speed = 250*DIMENSION_FACTOR;
        wp_req.llegada = false;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 1720;  //FINAL PISTA 32L
        wp_req.y = -5000;
        wp_req.z = 0;
        wp_req.speed = 150*DIMENSION_FACTOR;
        wp_req.llegada = false;
        srv.request.wps.push_back(wp_req); 
        if(highspeed == 1){  //L3
          wp_req.x = 1450;
          wp_req.y = -4800;
          wp_req.z = 0;
          wp_req.speed = 65*DIMENSION_FACTOR;
          wp_req.llegada = false;
          srv.request.wps.push_back(wp_req);
        }else if(highspeed == 2){  //L1
          wp_req.x = 1450;
          wp_req.y = -4250;
          wp_req.z = 0;
          wp_req.speed = 150*DIMENSION_FACTOR;
          wp_req.llegada = false;
          srv.request.wps.push_back(wp_req);
          wp_req.x = 1170;
          wp_req.y = -4050;
          wp_req.z = 0;
          wp_req.speed = 65*DIMENSION_FACTOR;
          wp_req.llegada = false;
          srv.request.wps.push_back(wp_req);
          wp_req.x = 1450;
          wp_req.y = -4800;
          wp_req.z = 0;
          wp_req.speed = 40*DIMENSION_FACTOR;
          wp_req.llegada = false;
          srv.request.wps.push_back(wp_req);
        }
        wp_req.x = 1520; //RODADURA POR A
        wp_req.y = -5000;
        wp_req.z = 0;
        wp_req.speed = 40*DIMENSION_FACTOR;
        wp_req.llegada = false;
        srv.request.wps.push_back(wp_req);
        if(srv2.response.terminal == 3){
          wp_req.x = 1750; //RODADURA POR A
          wp_req.y = -5600; 
          wp_req.z = 0;
          wp_req.speed = 40*DIMENSION_FACTOR;
          wp_req.llegada = false;
          srv.request.wps.push_back(wp_req);
          wp_req.x = 1660; //LLEGADA A T123
          wp_req.y = -5900; 
          wp_req.z = 0;
          wp_req.speed = 40*DIMENSION_FACTOR;
          wp_req.llegada = true;
          srv.request.wps.push_back(wp_req);
        }else{
          wp_req.x = 1430; //CAMBIO DE SENTIDO HACIA M
          wp_req.y = -5030;
          wp_req.z = 0;
          wp_req.speed = 40*DIMENSION_FACTOR;
          wp_req.llegada = false;
          srv.request.wps.push_back(wp_req);
          wp_req.x = 880; //RODADURA POR M CRITICAL POINT
          wp_req.y = -3450;
          wp_req.z = 0;
          wp_req.speed = 40*DIMENSION_FACTOR;
          wp_req.llegada = false;
          srv.request.wps.push_back(wp_req);
          if(srv2.response.terminal == 1){
            wp_req.x = 960; //RODADURA POR M CRITICAL POINT
            wp_req.y = -3200;
            wp_req.z = 0;
            wp_req.speed = 40*DIMENSION_FACTOR;
            wp_req.llegada = false;
            srv.request.wps.push_back(wp_req);
            wp_req.x = 2120; //RODADURA POR M CRITICAL POINT
            wp_req.y = -3580;
            wp_req.z = 0;
            wp_req.speed = 40*DIMENSION_FACTOR;
            wp_req.llegada = false;
            srv.request.wps.push_back(wp_req);
            wp_req.x = 2200; //LLEGADA A T4S
            wp_req.y = -3300;
            wp_req.z = 0;
            wp_req.speed = 40*DIMENSION_FACTOR;
            wp_req.llegada = true;
            srv.request.wps.push_back(wp_req);
          }else if(srv2.response.terminal == 2){
            wp_req.x = 200; //LLEGADA A T4
            wp_req.y = -3230;
            wp_req.z = 0;
            wp_req.speed = 40*DIMENSION_FACTOR;
            wp_req.llegada = true;
            srv.request.wps.push_back(wp_req);
          }
        }

      }else if(srv2.response.a_runaway == 4){  //ARRIVALS 32R
        wp_req.x = 5200;
        wp_req.y = -9000;                                
        wp_req.z = 300;
        wp_req.speed = v*DIMENSION_FACTOR;
        wp_req.llegada = false;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 4420;  //INICIO PISTA 32R
        wp_req.y = -6780;
        wp_req.z = 100;
        wp_req.speed = 250*DIMENSION_FACTOR;
        wp_req.llegada = false;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 3710;  //GROUND
        wp_req.y = -4800;
        wp_req.z = 0;
        wp_req.speed = 250*DIMENSION_FACTOR;
        wp_req.llegada = false;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 3450;  //FINAL PISTA 32R
        wp_req.y = -4080;
        wp_req.z = 0;
        wp_req.speed = 150*DIMENSION_FACTOR;
        wp_req.llegada = false;
        srv.request.wps.push_back(wp_req);
        if(highspeed == 1){  //K4
          wp_req.x = 3280;
          wp_req.y = -3600;
          wp_req.z = 0;
          wp_req.speed = 150*DIMENSION_FACTOR;
          wp_req.llegada = false;
          srv.request.wps.push_back(wp_req);
          wp_req.x = 2950;
          wp_req.y = -3350;
          wp_req.z = 0;
          wp_req.speed = 65*DIMENSION_FACTOR;
          wp_req.llegada = false;
          srv.request.wps.push_back(wp_req);
        }else if(highspeed == 2){  //K5
          wp_req.x = 3150;
          wp_req.y = -3890;
          wp_req.z = 0;
          wp_req.speed = 150*DIMENSION_FACTOR;
          wp_req.llegada = false;
          srv.request.wps.push_back(wp_req);
          wp_req.x = 3050;
          wp_req.y = -3600;
          wp_req.z = 0;
          wp_req.speed = 65*DIMENSION_FACTOR;
          wp_req.llegada = false;
          srv.request.wps.push_back(wp_req);
        }
        wp_req.x = 2900;  //RODADURA POR KA
        wp_req.y = -3200;
        wp_req.z = 0;
        wp_req.speed = 40*DIMENSION_FACTOR;
        wp_req.llegada = false;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 2670;  //CRITICAL POINT
        wp_req.y = -3120;
        wp_req.z = 0;
        wp_req.speed = 40*DIMENSION_FACTOR;
        wp_req.llegada = false;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 2500;  //RODADURA POR KC
        wp_req.y = -3080;
        wp_req.z = 0;
        wp_req.speed = 40*DIMENSION_FACTOR;
        wp_req.llegada = false;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 2340;  //CRITICAL POINT
        wp_req.y = -3650;
        wp_req.z = 0;
        wp_req.speed = 40*DIMENSION_FACTOR;
        wp_req.llegada = false;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 2310;  //RODADURA POR A
        wp_req.y = -3750;
        wp_req.z = 0;
        wp_req.speed = 40*DIMENSION_FACTOR;
        wp_req.llegada = false;
        srv.request.wps.push_back(wp_req); 
        if(srv2.response.terminal == 1){
          wp_req.x = 2080; //RODADURA POR A
          wp_req.y = -3680;
          wp_req.z = 0;
          wp_req.speed = 40*DIMENSION_FACTOR;
          wp_req.llegada = false;
          srv.request.wps.push_back(wp_req);
          wp_req.x = 2120; //CRITICAL POINT
          wp_req.y = -3580;
          wp_req.z = 0;
          wp_req.speed = 40*DIMENSION_FACTOR;
          wp_req.llegada = false;
          srv.request.wps.push_back(wp_req);
          wp_req.x = 2200; //LLEGADA A T4S
          wp_req.y = -3300;
          wp_req.z = 0;
          wp_req.speed = 40*DIMENSION_FACTOR;
          wp_req.llegada = true;
          srv.request.wps.push_back(wp_req);
        }else{
          wp_req.x = 1000;  //RODADURA POR A
          wp_req.y = -3320;
          wp_req.z = 0;
          wp_req.speed = 40*DIMENSION_FACTOR;
          wp_req.llegada = false;
          srv.request.wps.push_back(wp_req);
          wp_req.x = 950;  //RODADURA POR A
          wp_req.y = -3480;
          wp_req.z = 0;
          wp_req.speed = 40*DIMENSION_FACTOR;
          wp_req.llegada = false;
          srv.request.wps.push_back(wp_req);
          if(srv2.response.terminal == 2){
            wp_req.x = 880;  //CRITICAL POINT
            wp_req.y = -3450;
            wp_req.z = 0;
            wp_req.speed = 40*DIMENSION_FACTOR;
            wp_req.llegada = false;
            srv.request.wps.push_back(wp_req);
            wp_req.x = 200; //LLEGADA A T4
            wp_req.y = -3230;
            wp_req.z = 0;
            wp_req.speed = 40*DIMENSION_FACTOR;
            wp_req.llegada = true;
            srv.request.wps.push_back(wp_req);
          }else if(srv2.response.terminal == 3){
            wp_req.x = 1170;  //RODADURA POR A   CRITICAL POINT 1
            wp_req.y = -4050;
            wp_req.z = 0;
            wp_req.speed = 40*DIMENSION_FACTOR;
            wp_req.llegada = false;
            srv.request.wps.push_back(wp_req);
            wp_req.x = 1450;  //RODADURA POR A   CRITICAL POINT 2
            wp_req.y = -4800;
            wp_req.z = 0;
            wp_req.speed = 40*DIMENSION_FACTOR;
            wp_req.llegada = false;
            srv.request.wps.push_back(wp_req);
            wp_req.x = 1750; //RODADURA POR A
            wp_req.y = -5600; 
            wp_req.z = 0;
            wp_req.speed = 40*DIMENSION_FACTOR;
            wp_req.llegada = false;
            srv.request.wps.push_back(wp_req);
            wp_req.x = 1660; //LLEGADA A T123
            wp_req.y = -5900; 
            wp_req.z = 0;
            wp_req.speed = 40*DIMENSION_FACTOR;
            wp_req.llegada = true;
            srv.request.wps.push_back(wp_req);
          }
        }
      }

      //DEPARTURES

      if(srv2.response.d_runaway == 1){  //DEPARTURES 36L
        if(srv2.response.terminal == 1){
          wp_req.x = 2230; //RODADURA POR B
          wp_req.y = -3200; 
          wp_req.z = 0;
          wp_req.speed = 40*DIMENSION_FACTOR;
          wp_req.llegada = true;
          srv.request.wps.push_back(wp_req); 
          wp_req.x = 1620; //RODADURA POR B
          wp_req.y = -3020; 
          wp_req.z = 0;
          wp_req.speed = 40*DIMENSION_FACTOR;
          wp_req.llegada = true;
          srv.request.wps.push_back(wp_req); 
        }else if(srv2.response.terminal == 2){
          wp_req.x = 420; //RODADURA POR R
          wp_req.y = -2520; 
          wp_req.z = 0;
          wp_req.speed = 40*DIMENSION_FACTOR;
          wp_req.llegada = true;
          srv.request.wps.push_back(wp_req); 
        }else if(srv2.response.terminal == 3){
          wp_req.x = 1500;  //RODADURA POR M
          wp_req.y = -5850;
          wp_req.z = 0;
          wp_req.speed = 40*DIMENSION_FACTOR;
          wp_req.llegada = true;
          srv.request.wps.push_back(wp_req);
          wp_req.x = 1600; //RODADURA POR M
          wp_req.y = -5500;
          wp_req.z = 0;
          wp_req.speed = 40*DIMENSION_FACTOR;
          wp_req.llegada = true;
          srv.request.wps.push_back(wp_req);
          wp_req.x = 1430; //RODADURA POR M CRITICAL POINT
          wp_req.y = -5030;
          wp_req.z = 0;
          wp_req.speed = 40*DIMENSION_FACTOR;
          wp_req.llegada = true;
          srv.request.wps.push_back(wp_req);
          wp_req.x = 880; //RODADURA POR M CRITICAL POINT
          wp_req.y = -3450;
          wp_req.z = 0;
          wp_req.speed = 40*DIMENSION_FACTOR;
          wp_req.llegada = true;
          srv.request.wps.push_back(wp_req);
          wp_req.x = 960; //RODADURA POR M CRITICAL POINT
          wp_req.y = -3200;    
          wp_req.z = 0;
          wp_req.speed = 40*DIMENSION_FACTOR;
          wp_req.llegada = true;
          srv.request.wps.push_back(wp_req);
          wp_req.x = 2120;  //CRITICAL POINT
          wp_req.y = -3580;
          wp_req.z = 0;
          wp_req.speed = 40*DIMENSION_FACTOR;
          wp_req.llegada = true;
          srv.request.wps.push_back(wp_req);
          wp_req.x = 2230;  //RODADURA POR M
          wp_req.y = -3210;
          wp_req.z = 0;
          wp_req.speed = 40*DIMENSION_FACTOR;
          wp_req.llegada = true;
          srv.request.wps.push_back(wp_req);
          wp_req.x = 1620;  //RODADURA POR M
          wp_req.y = -3020;
          wp_req.z = 0;
          wp_req.speed = 40*DIMENSION_FACTOR;
          wp_req.llegada = true;
          srv.request.wps.push_back(wp_req);
          
        }
        wp_req.x = 1650; //CRITICAL POINT
        wp_req.y = -2900;
        wp_req.z = 0;
        wp_req.speed = 40*DIMENSION_FACTOR;
        wp_req.llegada = true;
        srv.request.wps.push_back(wp_req); 
        wp_req.x = 1800; //ENTRADA EN 36L
        wp_req.y = -2400;
        wp_req.z = 0;
        wp_req.speed = 40*DIMENSION_FACTOR;
        wp_req.llegada = true;
        srv.request.wps.push_back(wp_req); 
        wp_req.x = 2400; //SALIDA DE 36L
        wp_req.y = -450; 
        wp_req.z = 0;
        wp_req.speed = 250*DIMENSION_FACTOR;
        wp_req.llegada = true;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 6000; //PUNTO FINAL        
        wp_req.y = 20000;
        wp_req.z = 4000;
        wp_req.speed = 400*DIMENSION_FACTOR;    
        wp_req.llegada = true;  
        srv.request.wps.push_back(wp_req);

      }else if(srv2.response.d_runaway == 2){  //SALIDAS 36R
        if(srv2.response.terminal == 1){
          wp_req.x = 2230; //RODADURA POR B
          wp_req.y = -3200; 
          wp_req.z = 0;
          wp_req.speed = 40*DIMENSION_FACTOR;
          wp_req.llegada = true;
          srv.request.wps.push_back(wp_req);
          wp_req.x = 1930; //RODADURA POR B
          wp_req.y = -3110; 
          wp_req.z = 0;
          wp_req.speed = 40*DIMENSION_FACTOR;
          wp_req.llegada = true;
          srv.request.wps.push_back(wp_req);
          wp_req.x = 2230;  //RODADURA POR B
          wp_req.y = -2130;
          wp_req.z = 0;
          wp_req.speed = 40*DIMENSION_FACTOR;
          wp_req.llegada = true;
          srv.request.wps.push_back(wp_req);
        }else{
          if(srv2.response.terminal == 3){
            wp_req.x = 1500;  //RODADURA POR M
            wp_req.y = -5850;
            wp_req.z = 0;
            wp_req.speed = 40*DIMENSION_FACTOR;
            wp_req.llegada = true;
            srv.request.wps.push_back(wp_req);
            wp_req.x = 1600; //RODADURA POR M
            wp_req.y = -5500;
            wp_req.z = 0;
            wp_req.speed = 40*DIMENSION_FACTOR;
            wp_req.llegada = true;
            srv.request.wps.push_back(wp_req);
            wp_req.x = 1430; //RODADURA POR M CRITICAL POINT
            wp_req.y = -5030;
            wp_req.z = 0;
            wp_req.speed = 40*DIMENSION_FACTOR;
            wp_req.llegada = true;
            srv.request.wps.push_back(wp_req);
            wp_req.x = 880; //RODADURA POR M
            wp_req.y = -3450;
            wp_req.z = 0;
            wp_req.speed = 40*DIMENSION_FACTOR;
            wp_req.llegada = true;
            srv.request.wps.push_back(wp_req);
            wp_req.x = 950; //RODADURA POR M
            wp_req.y = -3200;    
            wp_req.z = 0;
            wp_req.speed = 40*DIMENSION_FACTOR;
            wp_req.llegada = true;
            srv.request.wps.push_back(wp_req);
          }else if(srv2.response.terminal == 2){
            wp_req.x = 280; //RODADURA POR M
            wp_req.y = -2980;    
            wp_req.z = 0;
            wp_req.speed = 40*DIMENSION_FACTOR;
            wp_req.llegada = true;
            srv.request.wps.push_back(wp_req);
          }
          wp_req.x = 960;  //CRITICAL POINT
          wp_req.y = -3200;
          wp_req.z = 0;
          wp_req.speed = 40*DIMENSION_FACTOR;
          wp_req.llegada = true;
          srv.request.wps.push_back(wp_req);
          wp_req.x = 2120;  //CRITICAL POINT
          wp_req.y = -3580;
          wp_req.z = 0;
          wp_req.speed = 40*DIMENSION_FACTOR;
          wp_req.llegada = true;
          srv.request.wps.push_back(wp_req);
          wp_req.x = 2340;  //CRITICAL POINT
          wp_req.y = -3650;
          wp_req.z = 0;
          wp_req.speed = 40*DIMENSION_FACTOR;
          wp_req.llegada = true;
          srv.request.wps.push_back(wp_req);
          wp_req.x = 2500; //RODADURA POR M
          wp_req.y = -3700;
          wp_req.z = 0;
          wp_req.speed = 40*DIMENSION_FACTOR;
          wp_req.llegada = true;
          srv.request.wps.push_back(wp_req);
          wp_req.x = 2670; //RODADURA POR M CRITICAL POINT
          wp_req.y = -3120;
          wp_req.z = 0;
          wp_req.speed = 40*DIMENSION_FACTOR;
          wp_req.llegada = true;
          srv.request.wps.push_back(wp_req);
        }
        wp_req.x = 2910; //RODADURA POR M CRITICAL POINT
        wp_req.y = -2350;
        wp_req.z = 0;
        wp_req.speed = 40*DIMENSION_FACTOR;
        wp_req.llegada = true;
        srv.request.wps.push_back(wp_req);
        wp_req.x = 3150; //ENTRADA EN 36R
        wp_req.y = -2425;
        wp_req.z = 0;
        wp_req.speed = 40*DIMENSION_FACTOR;    
        wp_req.llegada = true;  
        srv.request.wps.push_back(wp_req);
        wp_req.x = 3750; //SALIDA DE 36R
        wp_req.y = -500;
        wp_req.z = 0;
        wp_req.speed = 250*DIMENSION_FACTOR;    
        wp_req.llegada = true;  
        srv.request.wps.push_back(wp_req);
        wp_req.x = 14000; 
        wp_req.y = 20000;
        wp_req.z = 4000;
        wp_req.speed = 400*DIMENSION_FACTOR;    
        wp_req.llegada = true;  
        srv.request.wps.push_back(wp_req);
      }

      ROS_INFO_STREAM("NEW FLIGHT:  "<< id);

      if(srv2.response.generate != false){
        type_count++;
        num++; 
        if (client.call(srv)){
          if(!(srv.response.achieved))
          ROS_INFO_STREAM("FAIL: "<<srv.response.expl);
        }else{
          ROS_ERROR("Failed to call service");
          return 1;
        }
      }
    }
  }
  return 0;
}