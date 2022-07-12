/*
 * Airport.cpp
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
#include "atcsim_msgs/CommanderService.h"
#include "atcsim_msgs/InfoService.h"
#include "atcsim_msgs/ArrayFlight.h"
#include "atcsim_msgs/Controller.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sstream>
#include <string>
#include <cstdlib>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "Airport.h"


geometry_msgs::TransformStamped
generate_flight_obj(Position pos, float bearing, float inclination, std::string name_flight){

  tf2::Stamped<tf2::Transform> object;
  object.frame_id_ = "world";
  object.stamp_ = ros::Time::now();

  object.setOrigin(tf2::Vector3(pos.get_x(), pos.get_y(), pos.get_z()));

  tf2::Quaternion q;
  q.setRPY(0, -inclination, bearing);
  object.setRotation(q);

  geometry_msgs::TransformStamped object_msg = tf2::toMsg(object);
  object_msg.child_frame_id = name_flight;

  return object_msg;

}

void
Airport::drawInfo(){

  float cos45= 0.7071;
  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker marker_text_time;
  marker_text_time.header.frame_id = "world";
  marker_text_time.header.stamp = ros::Time::now();
  marker_text_time.ns = "Time";
  marker_text_time.id = 11;
  marker_text_time.action = visualization_msgs::Marker::ADD;
  marker_text_time.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  std::ostringstream buff2;
  float mins = (local_time_- fmod(local_time_,60))/60;
  float secs = fmod(local_time_,60);
  float decs = fmod(fmod(local_time_,60),1);
  if(mins<10 && secs<10){buff2<<"0"<<mins<<":0"<<(secs-decs);}
  else if(mins<10 && secs>=10){buff2<<"0"<<mins<<":"<<(secs-decs);}
  else if(mins>=10 && secs<10){buff2<<mins<<":0"<<(secs-decs);}
  else{buff2<<mins<<":"<<(secs-decs);}

  marker_text_time.text = buff2.str();
  marker_text_time.pose.position.x = 16000;
  marker_text_time.pose.position.y = -7000;
  marker_text_time.pose.position.z = 1000;
  marker_text_time.pose.orientation.x = 0.0;
  marker_text_time.pose.orientation.y = 0.0;
  marker_text_time.pose.orientation.z = 0.0;
  marker_text_time.pose.orientation.w = 1.0;
  marker_text_time.scale.z = 1500;
  marker_text_time.color.r = 0.0f;
  marker_text_time.color.g = 0.0f;
  marker_text_time.color.b = 0.0f;
  marker_text_time.color.a = 1.0;
  marker_array.markers.push_back(marker_text_time);

  airportMarker_pub_.publish(marker_array);

}

visualization_msgs::Marker
Airport::add_marker(std::string id, std::string type, int count, int mir){

  visualization_msgs::Marker aux_marker;
  aux_marker.header.frame_id =  id;
	aux_marker.header.stamp = ros::Time();
	aux_marker.ns = id;
	aux_marker.id = count;
	aux_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	aux_marker.action = visualization_msgs::Marker::ADD;
	aux_marker.pose.position.x = 0;
	aux_marker.pose.position.y = 0;
	aux_marker.pose.position.z = 0;
	aux_marker.pose.orientation.x = 0.0;
	aux_marker.pose.orientation.y = 0.0;
	aux_marker.pose.orientation.z = 1.0;
	aux_marker.pose.orientation.w = 1.0;
  ros::Duration one_second(1);
  aux_marker.lifetime = one_second;
  aux_marker.scale.x = 8*mir;
  aux_marker.scale.y = 8;
  aux_marker.scale.z = 8;
  aux_marker.color.a = 1.0;
  aux_marker.color.r = 0.831;
  aux_marker.color.g = 0.686;
  aux_marker.color.b = 0.216;
	aux_marker.mesh_resource = "package://atcsim/models/SC_Private_001.dae";

  return aux_marker;

}

visualization_msgs::Marker
Airport::drawWp(boost::shared_ptr<Flight> flight, int count){

  std::list<Route>::iterator it_wps;
  geometry_msgs::Point aux_point;
  visualization_msgs::Marker aux_marker;
  aux_marker.header.frame_id =  "world";
	aux_marker.header.stamp = ros::Time();
  std::ostringstream buff2;
  buff2<<(flight)->getId()<<"_wps";
	aux_marker.ns = buff2.str();
	aux_marker.id = count;
	aux_marker.type = visualization_msgs::Marker::LINE_STRIP;
	aux_marker.action = visualization_msgs::Marker::ADD;
	aux_marker.pose.position.x = 0;
	aux_marker.pose.position.y = 0;
	aux_marker.pose.position.z = 0;
	aux_marker.pose.orientation.x = 0.0;
	aux_marker.pose.orientation.y = 0.0;
	aux_marker.pose.orientation.z = 0.0;
	aux_marker.pose.orientation.w = 1.0;
  ros::Duration one_second(1);
  aux_marker.lifetime = one_second;
	aux_marker.scale.x = 20;
	aux_marker.color.a = 1.0;
	aux_marker.color.r = 0.0;
	aux_marker.color.g = 0.0;
	aux_marker.color.b = 1.0;

  aux_point.x = flight->getPosition().get_x();
  aux_point.y = flight->getPosition().get_y();
  aux_point.z = flight->getPosition().get_z();
  aux_marker.points.push_back(aux_point);
  if(flight->getHandling() == 0){
    it_wps = (((flight)->getRoute())->begin());
    for(int i = 0; i <= flight->getNWaypArrive(); ++i){
      aux_point.x = it_wps->pos.get_x();
      aux_point.y = it_wps->pos.get_y();
      aux_point.z = it_wps->pos.get_z();
      aux_marker.points.push_back(aux_point);
      ++it_wps;
    }
  }else if(flight->getHandling() == 2){
    for(it_wps = (((flight)->getRoute())->begin()); it_wps!=((flight)->getRoute())->end(); ++it_wps){
     aux_point.x = it_wps->pos.get_x();
     aux_point.y = it_wps->pos.get_y();
     aux_point.z = it_wps->pos.get_z();
     aux_marker.points.push_back(aux_point);
    }
  }
  return aux_marker;

}


visualization_msgs::Marker
Airport::add_marker_sphere_flight(std::string id,std::string type, int count){

  visualization_msgs::Marker aux_marker;
  std::ostringstream buff2;
  buff2<<id<<"_sphere";
  aux_marker.header.frame_id =  id;
	aux_marker.header.stamp = ros::Time();

	aux_marker.ns = buff2.str();
	aux_marker.id = count;
	aux_marker.type = visualization_msgs::Marker::SPHERE;
	aux_marker.action = visualization_msgs::Marker::ADD;
	aux_marker.pose.position.x = 0;
	aux_marker.pose.position.y = 0;
	aux_marker.pose.position.z = 0;
	aux_marker.pose.orientation.x = 0.0;
	aux_marker.pose.orientation.y = 0.0;
	aux_marker.pose.orientation.z = 1.0;
	aux_marker.pose.orientation.w = 1.0;
  ros::Duration one_seconds(1);
  aux_marker.lifetime = one_seconds;
  aux_marker.scale.x = A320_size;
  aux_marker.scale.y = A320_size;
  aux_marker.scale.z = A320_size;
  aux_marker.color.a = 0.5;
  aux_marker.color.r = 1;
  aux_marker.color.g = 0.0;
  aux_marker.color.b = 0.0;
  return aux_marker;

}

void
Airport::handling(){

  std::list<boost::shared_ptr<Flight> >::iterator it;
  ros::Duration handling_duration(HANDLING_TIME);
  Results i;

  for(it = flights_.begin(); it!=flights_.end(); ++it){
    if((*it)->atTerminal() && (*it)->getHandling() == 0){
      (*it)->setHandling();
      (*it)->setArrival_Time(ros::Time::now());
      (*it)->setDeparture_Time((*it)->getArrival_Time() + handling_duration);
      if(local_time_ >= 20*60){
        arrivals++;
        pax_a = pax_a + ((*it)->getCapacity() * (*it)->getOccupation()) / 100;
        i.airline = (*it)->getAirline();
        i.pax = (*it)->getCapacity() * (*it)->getOccupation() / 100;
        i.type = (*it)->getType();
        i.id = (*it)->getId();
        arrivals_list.push_back(i);
        if((*it)->getTerminal() == 1){
          T4S++;
        }else if((*it)->getTerminal() == 2){
          T4++;
        }else{
          T123++;
        }

        if((*it)->getRunaway_arrival() == 3){
          _32L++;
        }else{
          _32R++;
        }

      }
    }else if((*it)->getHandling() == 1 && ros::Time::now() > (*it)->getDeparture_Time()){
      (*it)->setHandling();
    }
  }

}

void
Airport::check_waiting(){

  atcsim_msgs::Controller cnt_pub;
  bool waiting = false;
  std::list<boost::shared_ptr<Flight> >::iterator it;
  int i, j;
  i = 0;
  j = 0;

  for(it = flights_.begin(); it!=flights_.end(); ++it){
    if((*it)->getRunaway_arrival() == 3 && (*it)->getRoute()->front().waiting == true){
      i++;
    }else if((*it)->getRunaway_arrival() == 4 && (*it)->getRoute()->front().waiting == true){
      j++;
    }
  }

  if(i >= 1){
    cnt_pub.L_waiting = true;
  }else{
    cnt_pub.L_waiting = false;
  }

  if(j >= 1){
    cnt_pub.R_waiting = true;
  }else{
    cnt_pub.R_waiting = false;
  }

  p_arival_info_pub.publish(cnt_pub);

  return;

}

void 
Airport::check_departures(){

  Results i;
  Position P0;
  P0.set_x(2500);
  P0.set_y(-3000);
  P0.set_z(0);
  std::list<boost::shared_ptr<Flight> >::iterator it;

  for(it = flights_.begin(); it!=flights_.end(); ++it){
    if((*it)->getPosition().distanceXY(P0) > 20000){
      if(local_time_ >= 20*60){
        departures++;
        pax_d = pax_d + ((*it)->getCapacity() * (*it)->getOccupation()) / 100;
        i.airline = (*it)->getAirline();
        i.pax = (*it)->getCapacity() * (*it)->getOccupation() / 100;
        i.type = (*it)->getType();
        i.id = (*it)->getId();
        departures_list.push_back(i);

        if((*it)->getRunaway_departure() == 1){
          _36L++;
        }else{
          _36R++;
        }
      }
      flights_.erase(it);
      return;
    }
  }

}

void
Airport::results(){

  if(local_time_ >= 40*60.0 && local_time_ <= (40*60.0 + 0.1)){
    std::cerr<<"-----ARRIVALS-----"<<std::endl;
    std::cerr<<"Nº operations = "<<arrivals<<std::endl;
    std::cerr<<"Pax. = "<<pax_a<<std::endl<<std::endl;
    std::cerr<<"List"<<std::endl;
    int j = arrivals_list.size();
    for(int i = 0; i < j; i++){
      std::cerr<<"AIRLINE = "<<arrivals_list.front().airline;
      std::cerr<<"  TYPE = "<<arrivals_list.front().type;
      std::cerr<<"  PAX = "<<arrivals_list.front().pax<<std::endl;
      std::cerr<<arrivals_list.front().id<<std::endl;
      arrivals_list.pop_front();
    }
    std::cerr<<std::endl<<std::endl;
    std::cerr<<"T123 = "<<T123<<std::endl;
    std::cerr<<"T4 = "<<T4<<std::endl;
    std::cerr<<"T4S = "<<T4S<<std::endl;
    std::cerr<<"32L = "<<_32L<<std::endl;
    std::cerr<<"32R = "<<_32R<<std::endl;

    std::cerr<<std::endl<<std::endl;
    std::cerr<<"-----DEPARTURES-----"<<std::endl;
    std::cerr<<"Nº operations = "<<departures<<std::endl;
    std::cerr<<"Pax. = "<<pax_d<<std::endl<<std::endl;
    std::cerr<<"List"<<std::endl;
    int k = departures_list.size();
    for(int i = 0; i < k; i++){
      std::cerr<<departures_list.front().airline;
      std::cerr<<"  TYPE = "<<departures_list.front().type;
      std::cerr<<"  PAX = "<<departures_list.front().pax<<std::endl;
      std::cerr<<departures_list.front().id<<std::endl;
      departures_list.pop_front();
    }
    std::cerr<<"36L = "<<_36L<<std::endl;
    std::cerr<<"36R = "<<_36R<<std::endl;
  }

}

void
Airport::doWork(){

  std::list<boost::shared_ptr<Flight> >::iterator it, it2;
  std::list<boost::shared_ptr<visualization_msgs::Marker> >::iterator it_marker;
  std::list<Route>::iterator it_wps;

  atcsim_msgs::ArrayFlight arrayflight_pub;
  atcsim_msgs::Flight flight_pub;
  atcsim_msgs::Waypoint wp_pub;

  ros::Publisher arrayflight_info_pub_;
  arrayflight_info_pub_ = n_.advertise<atcsim_msgs::ArrayFlight>("arrayflight_info", 1000);
  ros::Rate loop_rate(10);

  drawInfo();

  if ((ros::Time::now() - obj_ts_).toSec() > INC_SIMTIME*mult_speed_/50){                 
    local_time_ += INC_SIMTIME;
    obj_ts_ = ros::Time::now();
    arrayflight_pub.array_flight.clear();

    for(it = flights_.begin(); it!=flights_.end(); ++it){
      bool wait = false;
      for(it2 = flights_.begin(); it2!=it; ++it2){
        if((*it)->at_crash_point() == 3 && (*it2)->at_crash_point() == 3){
          if((*it)->getPosition().distanceXY((*it)->getRoute()->front().pos) > 50 && (*it)->getPosition().distanceXY((*it)->getRoute()->front().pos) < 300 && (*it2)->getPosition().distanceXY((*it2)->getRoute()->front().pos) < 300){
            wait = true;
          }
        }else if((*it)->at_crash_point() == 4 && (*it2)->at_crash_point() == 4){
          if((*it)->getPosition().distanceXY((*it)->getRoute()->front().pos) > 50 && (*it)->getPosition().distanceXY((*it)->getRoute()->front().pos) < 300 && (*it2)->getPosition().distanceXY((*it2)->getRoute()->front().pos) < 300){
            wait = true;
          }
        }else if((*it)->at_crash_point() == 5 && (*it2)->at_crash_point() == 5){
          if((*it)->getPosition().distanceXY((*it)->getRoute()->front().pos) > 50 && (*it)->getPosition().distanceXY((*it)->getRoute()->front().pos) < 300 && (*it2)->getPosition().distanceXY((*it2)->getRoute()->front().pos) < 300){
            wait = true;
          }
        }else if((*it)->at_crash_point() == 6 && (*it2)->at_crash_point() == 6){
          if((*it)->getPosition().distanceXY((*it)->getRoute()->front().pos) > 50 && (*it)->getPosition().distanceXY((*it)->getRoute()->front().pos) < 300 && (*it2)->getPosition().distanceXY((*it2)->getRoute()->front().pos) < 300){
            wait = true;
          }
        }else if((*it)->at_crash_point() == 7 && (*it2)->at_crash_point() == 7){
          if((*it)->getPosition().distanceXY((*it)->getRoute()->front().pos) > 50 && (*it)->getPosition().distanceXY((*it)->getRoute()->front().pos) < 300 && (*it2)->getPosition().distanceXY((*it2)->getRoute()->front().pos) < 300){
            wait = true;
          }
        }else if((*it)->at_crash_point() == 8 && (*it2)->at_crash_point() == 8){
          if((*it)->getPosition().distanceXY((*it)->getRoute()->front().pos) > 50 && (*it)->getPosition().distanceXY((*it)->getRoute()->front().pos) < 300 && (*it2)->getPosition().distanceXY((*it2)->getRoute()->front().pos) < 300){
            wait = true;
          }
        }else if((*it)->at_crash_point() == 9 && (*it2)->at_crash_point() == 9){
          if((*it)->getPosition().distanceXY((*it)->getRoute()->front().pos) > 50 && (*it)->getPosition().distanceXY((*it)->getRoute()->front().pos) < 300 && (*it2)->getPosition().distanceXY((*it2)->getRoute()->front().pos) < 300){
            wait = true;
          }
        }else if((*it)->at_crash_point() == 10 && (*it2)->at_crash_point() == 10){
          if((*it)->getPosition().distanceXY((*it)->getRoute()->front().pos) > 50 && (*it)->getPosition().distanceXY((*it)->getRoute()->front().pos) < 300 && (*it2)->getPosition().distanceXY((*it2)->getRoute()->front().pos) < 300){
            wait = true;
          }
        }
      }

      for(it2 = flights_.begin(); it2!=flights_.end(); ++it2){
        if((*it)->getId() != (*it2)->getId()){
          if((*it)->at_crash_point() == 1 && (*it2)->at_crash_point() == 1 && (*it)->getRunaway_arrival() == 4){
            if((*it)->getPosition().distanceXY((*it)->getRoute()->front().pos) > 50 && (*it)->getPosition().distanceXY((*it)->getRoute()->front().pos) < 300 && (*it2)->getPosition().distanceXY((*it2)->getRoute()->front().pos) < 300){
              wait = true;
            }
          }else if((*it)->at_crash_point() == 2 && (*it2)->at_crash_point() == 2 && (*it2)->getRunaway_arrival() == 3 && (*it2)->getHighspeed() == 1){
            if((*it)->getRunaway_arrival() == 3 && (*it)->getHighspeed() == 2 || (*it)->getRunaway_arrival() == 4){
              if((*it)->getPosition().distanceXY((*it)->getRoute()->front().pos) > 50 && (*it)->getPosition().distanceXY((*it)->getRoute()->front().pos) < 300 && (*it2)->getPosition().distanceXY((*it2)->getRoute()->front().pos) < 300){
                wait = true;
              }
            }
          }
        }
      }


      //ESPERAS EN TIERRA
      if((*it)->at_P0_Runaway()){
        for(it2 = flights_.begin(); it2!=flights_.end(); ++it2){
          if((*it)->getId() != (*it2)->getId()){
            if((*it2)->at_d_Runaway() && (*it)->getRunaway_departure() == (*it2)->getRunaway_departure()){ 
              wait = true;
            }
          }
        }
      }

      //ESPERAS EN AIRE
      if((*it)->at_waiting_zone() == true){
        for(it2 = flights_.begin(); it2!=it; ++it2){
          if((*it)->getRunaway_arrival() == (*it2)->getRunaway_arrival() && (*it2)->at_waiting_zone() == true){
            int t1, t2;
            t1 = (*it)->getPosition().distanceXY((*it)->getRoute()->front().pos) / (*it)->getSpeed();
            t2 = (*it2)->getPosition().distanceXY((*it2)->getRoute()->front().pos) / (*it2)->getSpeed();
            if(t1 < (t2 + 26)){
              (*it)->waiting_circ();
            }
          }
        }
      }    
      
      if(wait == false && (*it)->getHandling() != 1){
        (*it)->update(0.1);
      }

      flight_pub.id = (*it)->getId();
      flight_pub.type = (*it)->getType();
      flight_pub.posx = (*it)->getPosition().get_x();
      flight_pub.posy = (*it)->getPosition().get_y();
      flight_pub.posz = (*it)->getPosition().get_z();
      flight_pub.speed = (*it)->getSpeed();
      flight_pub.bearing = (*it)->getBearing();
      flight_pub.inclination = (*it)->getInclination();
      flight_pub.points = (*it)->getPoints();
      flight_pub.wps.clear();

      for(it_wps = (((*it)->getRoute())->begin()); it_wps!=((*it)->getRoute())->end(); ++it_wps){  
        wp_pub.x = it_wps->pos.get_x();
        wp_pub.y = it_wps->pos.get_y();
        wp_pub.z = it_wps->pos.get_z();
        wp_pub.speed = it_wps->speed;

        flight_pub.wps.push_back(wp_pub);
      }
      
      arrayflight_pub.array_flight.push_back(flight_pub);
      
      
    }
      arrayflight_info_pub_.publish(arrayflight_pub);
  }///--------------------------------------------------------------------------


  for(it = flights_.begin(); it!=flights_.end(); ++it)///-- TRANSFORM PUBLISHING
  {

    flight_obj_msg = generate_flight_obj((*it)->getPosition(), (*it)->getBearing(), (*it)->getInclination(), (*it)->getId());

    try
    {
      flight_obj_msg.header.stamp = ros::Time::now();
      br.sendTransform(flight_obj_msg);

    }
    catch(tf2::TransformException &exception)
    {
      ROS_ERROR("04");
      ROS_ERROR("%s", exception.what());
    }

  }

  visualization_msgs::MarkerArray marker_array;

  int count = 0;
  for(it = flights_.begin(); it!=flights_.end(); ++it)
  {
    //marker_array.markers.push_back(add_marker((*it)->getId(),(*it)->getType(),count++,1));
    //marker_array.markers.push_back(add_marker((*it)->getId(),(*it)->getType(),count++,-1));
    marker_array.markers.push_back(add_marker_sphere_flight((*it)->getId(),(*it)->getType(),count++));
    marker_array.markers.push_back(drawWp(*it,count++));
  }
  flightMarkers_pub_.publish(marker_array);

}

bool
Airport::command(atcsim_msgs::CommanderService::Request& req, atcsim_msgs::CommanderService::Response& res){

  std::list<boost::shared_ptr<Flight> >::iterator it;
  std::list<Route>::iterator it_wps;
  bool ach = true;
    
  for(it = flights_.begin(); it!=flights_.end(); ++it){  //IDs conflicts checking 
    if (((*it)->getId()) == req.id){
      ach = false;
    }
  }

  if(ach){
    Position ipos(req.posx, req.posy, req.posz);
    boost::shared_ptr<Flight> aux(new Flight(req.id, req.type, ipos, req.bearing, req.inclination, req.speed, req.terminal, req.a_runaway, req.d_runaway, req.highspeed, req.generation_time, req.airline, req.capacity, req.occupation));
    int n = 0;
    for(int i=0; i<req.wps.size(); ++i){
      Position pos0(req.wps[i].x, req.wps[i].y, req.wps[i].z);
      Route r0;
      r0.pos = pos0;
      r0.speed = req.wps[i].speed;
      aux->getRoute()->push_back(r0);
      if(req.wps[i].llegada==false){
        n++;
      }
    }
    aux->setNWaypArrive(n);
    flights_.push_back(aux);
  }

  res.achieved = ach;
  if(ach){
    ROS_INFO_STREAM(req.id <<" added to the list");
  }else{
    res.expl = "Flight " + req.id + " already exists";
    return 1;
  }
  return true;

}

