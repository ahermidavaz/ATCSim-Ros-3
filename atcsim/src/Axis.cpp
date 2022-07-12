/*
 * Axis.cpp
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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "Barajas.h"

void
Airport_shape::generate_axis()
{
  visualization_msgs::MarkerArray airport_AXIS;

  visualization_msgs::Marker EJEX;
  EJEX.header.frame_id = "world";
  EJEX.header.stamp = ros::Time::now();
  EJEX.ns = "X AXIS";
  EJEX.id = 28;
  EJEX.action = visualization_msgs::Marker::ADD;
  EJEX.type = visualization_msgs::Marker::LINE_STRIP;
  EJEX.scale.x = 50;
  EJEX.color.r = 1.0f;
  EJEX.color.a = 1.0;
  EJEX.pose.orientation.w = 1.0;
  geometry_msgs::Point p5, q5;
  p5.x=-30000;
  p5.y=0;
  p5.z=0;
  q5.x=30000;
  q5.y=0;
  q5.z=0;
  EJEX.points.push_back(p5);
  EJEX.points.push_back(q5);
  airport_AXIS.markers.push_back(EJEX);

  visualization_msgs::Marker EJEY;
  EJEY.header.frame_id = "world";
  EJEY.header.stamp = ros::Time::now();
  EJEY.ns = "Y AXIS";
  EJEY.id = 29;
  EJEY.action = visualization_msgs::Marker::ADD;
  EJEY.type = visualization_msgs::Marker::LINE_STRIP;
  EJEY.scale.x = 50;
  EJEY.color.b = 1.0f;
  EJEY.color.a = 1.0;
  EJEY.pose.orientation.w = 1.0;
  geometry_msgs::Point p6, q6;
  p6.x=0;
  p6.y=-30000;
  q6.x=0;
  q6.y=30000;
  EJEY.points.push_back(p6);
  EJEY.points.push_back(q6);
  airport_AXIS.markers.push_back(EJEY);


  visualization_msgs::Marker x_list;
  x_list.header.frame_id = "world";
  x_list.header.stamp = ros::Time::now();
  x_list.ns = "x_list";
  x_list.id = 30;
  x_list.action = visualization_msgs::Marker::ADD;
  x_list.type = visualization_msgs::Marker::LINE_LIST;
  x_list.scale.x = 10;
  x_list.color.r = 1.0f;
  x_list.color.a = 1.0;
  x_list.pose.orientation.w = 1.0;
  for(int i = -30; i<=30; ++i)
  {
    geometry_msgs::Point p7;
    p7.x = 30000;
    p7.y= i*1000;
    p7.z=0;

    x_list.points.push_back(p7);
    p7.x = -p7.x;
    x_list.points.push_back(p7);
  }
  airport_AXIS.markers.push_back(x_list);

  visualization_msgs::Marker y_list;
  y_list.header.frame_id = "world";
  y_list.header.stamp = ros::Time::now();
  y_list.ns = "y_list";
  y_list.id = 31;
  y_list.action = visualization_msgs::Marker::ADD;
  y_list.type = visualization_msgs::Marker::LINE_LIST;
  y_list.scale.x = 10;
  y_list.color.r = 1.0f;
  y_list.color.a = 1.0;
  y_list.pose.orientation.w = 1.0;
  for(int z = -30; z<=30; ++z)
  {
    geometry_msgs::Point p8;
    p8.x = z*1000;
    p8.y= 30000;
    p8.z=0;

    y_list.points.push_back(p8);
    p8.y = -p8.y;
    y_list.points.push_back(p8);
  }
  airport_AXIS.markers.push_back(y_list);

visualization_msgs::Marker x_list_dec;
  x_list_dec.header.frame_id = "world";
  x_list_dec.header.stamp = ros::Time::now();
  x_list_dec.ns = "x_list_dec";
  x_list_dec.id = 32;
  x_list_dec.action = visualization_msgs::Marker::ADD;
  x_list_dec.type = visualization_msgs::Marker::LINE_LIST;
  x_list_dec.scale.x = 2;
  x_list_dec.color.r = 1.0f;
  x_list_dec.color.a = 1.0;
  x_list_dec.pose.orientation.w = 1.0;
  for(int i = -300; i<=300; ++i)
  {
    geometry_msgs::Point p9;
    p9.x = 30000;
    p9.y= i*100;
    p9.z=0;

    x_list_dec.points.push_back(p9);
    p9.x = -p9.x;
    x_list_dec.points.push_back(p9);
  }
  airport_AXIS.markers.push_back(x_list_dec);

  visualization_msgs::Marker y_list_dec;
  y_list_dec.header.frame_id = "world";
  y_list_dec.header.stamp = ros::Time::now();
  y_list_dec.ns = "y_list_dec";
  y_list_dec.id = 33;
  y_list_dec.action = visualization_msgs::Marker::ADD;
  y_list_dec.type = visualization_msgs::Marker::LINE_LIST;
  y_list_dec.scale.x = 2;
  y_list_dec.color.r = 1.0f;
  y_list_dec.color.a = 1.0;
  y_list_dec.pose.orientation.w = 1.0;
  for(int i = -300; i<=300; ++i)
  {
    geometry_msgs::Point p10;
    p10.x = i*100;
    p10.y= 30000;
    p10.z=0;

    y_list_dec.points.push_back(p10);
    p10.y = -p10.y;
    y_list_dec.points.push_back(p10);
  }
  airport_AXIS.markers.push_back(y_list_dec);

  generate_axis_pub.publish(airport_AXIS);
}
