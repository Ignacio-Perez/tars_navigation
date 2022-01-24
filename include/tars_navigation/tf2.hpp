/*
MIT License

Copyright (c) 2021 Ignacio Pérez Hurtado de Mendoza

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef _TF2_HPP_
#define _TF2_HPP_

#include <string>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "vector2d.hpp"
#include "angle.hpp"

class Tf2
{
public:
	~Tf2() {}
	Tf2(const Tf2& other) = delete;
	void operator=(const Tf2& other) = delete;
	static Tf2& getInstance() {static Tf2 instance;return instance;}
	#define TF2 Tf2::getInstance()
	void transform(const nav_msgs::Odometry::ConstPtr& msg, utils::Vector2d& pos, utils::Angle& yaw, const std::string& frame ) const;

private:
	Tf2() : listener(buffer) {}
	tf2_ros::Buffer buffer;
	tf2_ros::TransformListener listener;

};



inline 
void Tf2::transform(const nav_msgs::Odometry::ConstPtr& msg, utils::Vector2d& pos, utils::Angle& angle, const std::string& frame ) const {
	geometry_msgs::PoseStamped poseIn;
	poseIn.header = msg->header;
	poseIn.pose = msg->pose.pose;
	geometry_msgs::PoseStamped poseOut = buffer.transform(poseIn,frame);
	pos.set(poseOut.pose.position.x,poseOut.pose.position.y);
	tf2::Quaternion q(poseOut.pose.orientation.x,poseOut.pose.orientation.y,poseOut.pose.orientation.z,poseOut.pose.orientation.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll,pitch,yaw);
	angle.setRadian(yaw);

}










#endif