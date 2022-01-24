/*
MIT License

Copyright (c) 2022 Ignacio Pérez Hurtado de Mendoza

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

#include <list>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <tars_navigation/tf2.hpp>
#include <tars_navigation/map.hpp>
#include <tars_navigation/rrt.hpp>
#include <tars_navigation/Navigation.h>


// Posibles estados del robot
enum State {INIT, COMPUTING_RRT, NAVIGATING};


// Estructura para almacenar toda la informacion sobre el robot
struct Robot
{
	State state;
	std::string id;  // Identificador del robot 
	double radius; // radio del robot (m)
	utils::Vector2d position; // Posicion del robot en el frame "map"
	utils::Angle yaw; // Angulo del robot
	ros::Publisher mapPublisher; //publicador del mapa de navegacion
	utils::Vector2d goal;
	bool goalReceived;
	std::list<utils::Vector2d> path;
	utils::Vector2d init;
	RRT rrt; // RRT
	ros::Publisher goalPublisher;
} robot;

/***************************/
/* DEFINICION DE FUNCIONES */
/***************************/

// Funcion main
int main(int argc, char** argv);

// Funcion para procesar la recepcion de odometria
void odomReceivedCallback(const nav_msgs::Odometry::ConstPtr& msg);

// Funcion para procesar la recepcion de un goal
void goalReceivedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

// Funcion para recibir el mapa
void mapReceivedCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

// Funcion para visualizar el path
void publishPath(ros::Publisher& pub);

// Servicio para iniciar la navegacion
bool initNavigation(tars_navigation::Navigation::Request  &req,tars_navigation::Navigation::Response &res);

// Servicio para finalizar la navegacion
bool endNavigation(tars_navigation::Navigation::Request  &req, tars_navigation::Navigation::Response &res);

/****************/
/* FUNCION MAIN */
/****************/
int main(int argc, char** argv) {

	/*******************/
	/* INICIALIZACION  */
	/*******************/
	// La inicializacion de ROS, es completamente necesaria antes de ejecutar ninguna otra funcion de ROS
	ros::init(argc,argv,"global_planning_node");

	// Definimos dos manejadores, uno publico y otro privado, para comunicarnos con el sistema ROS
	ros::NodeHandle n, pn("~");
	

	/*************************/
	/* LECTURA DE PARAMETROS */
	/*************************/

	// frecuencia del bucle principal
	double freq;
	pn.param<double>("freq",freq,20);
	
	// Identificador del robot
	pn.param<std::string>("robot_id",robot.id,"09");

	// Radio del robot (m)
	pn.param<double>("robot_radius",robot.radius,0.2);

	// topic para recibir odometria
	std::string odom_topic;
	pn.param<std::string>("odom_topic",odom_topic,"/tars/09/odom");

	// topic para recibir los goals
	std::string goal_topic;
	pn.param<std::string>("goal_topic",goal_topic,"/move_base_simple/goal");

	std::string local_goal_topic;
	pn.param<std::string>("local_goal_topic",local_goal_topic,"/tars/09/goal");

	// topic para recibir el mapa
	std::string map_topic;
	pn.param<std::string>("map_topic",map_topic,"/map");	

	// topic para publicar el mapa de navegacion
	std::string nav_map_topic;
	pn.param<std::string>("nav_map_topic",nav_map_topic,"/map2");

	// topic para publicar la visualizacion del path
	std::string path_topic;
	pn.param<std::string>("path_topic",path_topic,"/tars/visualization/09/path");

	// servicio para iniciar la navegacion
	std::string init_navigation_service;
	pn.param<std::string>("init_navigation_service",init_navigation_service,"/tars/09/init_navigation");

	// servicio para detener la navegacion
	std::string end_navigation_service;
	pn.param<std::string>("end_navigation_service",end_navigation_service,"/tars/09/end_navigation");

	/****************************/
	/* INICIALIZACION DEL ROBOT */
	/****************************/
	robot.state = INIT;
	robot.goalReceived = false;
	ROS_INFO("State: INIT");
	
	/*************************/
	/* SUBSCRIPCION A TOPICS */
	/*************************/

	// Topic para recibir la odometria
	ros::Subscriber odomSubscriber = n.subscribe<nav_msgs::Odometry>(odom_topic,1,odomReceivedCallback);
	
	// Topic para recibir los goals
	ros::Subscriber goalSubscriber = n.subscribe<geometry_msgs::PoseStamped>(goal_topic,1,goalReceivedCallback);

	// Topic para recibir el mapa
	ros::Subscriber mapSubscriber = n.subscribe<nav_msgs::OccupancyGrid>(map_topic,1,mapReceivedCallback);

	/*************************/
	/* PUBLICACION DE TOPICS */
	/*************************/

	// Topic para publicar el mapa de navegacion
	robot.mapPublisher = pn.advertise<nav_msgs::OccupancyGrid>(nav_map_topic,1,true);

	// Topic para publicar la visualizacion del path
	ros::Publisher pathPublisher = pn.advertise<visualization_msgs::Marker>(path_topic,1);

	// Topic para publicar los goals locales
	robot.goalPublisher = pn.advertise<geometry_msgs::PoseStamped>(local_goal_topic,1);

	/*************/
	/* SERVICIOS */
	/*************/
	ros::ServiceServer initNavService = n.advertiseService(init_navigation_service, initNavigation);
	ros::ServiceServer endNavService = n.advertiseService(end_navigation_service, endNavigation);

	/**********************/
 	/* BUCLE PRINCIPAL    */
 	/**********************/
	// La frecuencia a la que queremos que se ejecute el bucle
	ros::Rate r(freq);
	double computingTime = (1.0/freq)*0.9;


	// El Bucle principal, que funcionara idealmente a una tasa constante r
	while (n.ok()) {

		ros::Time prevTime = ros::Time::now();	

		while (robot.state == COMPUTING_RRT && (ros::Time::now() - prevTime).toSec() < computingTime) {
			robot.rrt.iterate();
		}

		if (robot.state == COMPUTING_RRT && robot.goalReceived) {
			robot.rrt.getPath(robot.goal,robot.path);
		}

		if (robot.state == COMPUTING_RRT || robot.state == NAVIGATING) {
			publishPath(pathPublisher);	
		}

		// El nodo se duerme para ajustarse a la frecuencia deseada
		r.sleep();

		// Se leen y procesan los mensajes que llegan por los topics
		ros::spinOnce();
	}

	return 0;
}

/************************/
/* FUNCIONES CALLBACK   */
/************************/

// Funcion para procesar la recepcion del mapa
void mapReceivedCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
	ROS_INFO("MAP RECEIVED");
	MAP.setData(msg,robot.radius);
	robot.mapPublisher.publish(MAP.getMap());
}


// Funcion para procesar la recepcion de odometria
void odomReceivedCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	try {
		TF2.transform(msg,robot.position,robot.yaw,"map");
		if (robot.state == INIT && MAP.isInitiated()) {
			ROS_INFO("ODOM RECEIVED X = %f Y = %f",robot.position.getX(),robot.position.getY());
			robot.state = COMPUTING_RRT;
			robot.init = robot.position;
			ROS_INFO("State: COMPUTING_RRT");
			robot.rrt.init(robot.position,robot.radius);
		} 
    } catch (tf2::TransformException &ex) {
		
    }
}


// Funcion para procesar la recepcion de un goal
void goalReceivedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	if (robot.state != COMPUTING_RRT) {
		return;
	}

	if (msg->header.frame_id != "map") {
		ROS_WARN("goalReceivedCallback: Invalid frame_id '%s'",msg->header.frame_id.c_str());
		return;
	}
	
	robot.goal.set(msg->pose.position.x, msg->pose.position.y);
	robot.goalReceived = true;
	ROS_INFO("GOAL RECEIVED X = %f Y = %f",robot.goal.getX(),robot.goal.getY());
}


/**************/
/* SERVICIOS */
/*************/
bool initNavigation(tars_navigation::Navigation::Request  &req,tars_navigation::Navigation::Response &res) {
	
	if (robot.state == NAVIGATING) {
		res.error = true;
		res.msg = "Already navigating";
	} else if (robot.state == INIT) {
		res.error = true;
		res.msg = "Not initiated";
	} else if (!robot.goalReceived) {
		res.error = true;
		res.msg = "Goal has not been received";
	} else if (robot.path.empty()) {
		res.error = true;
		res.msg = "Empty path";
	} else {
		robot.state = NAVIGATING;
		res.error = false;
		res.msg = "OK";
		geometry_msgs::PoseStamped goal;
		goal.header.frame_id = "map";
		goal.pose.position.z = 0;
		goal.pose.orientation.w = 1;
		for (auto it = robot.path.begin();it!=robot.path.end();++it) {
			goal.header.stamp = ros::Time::now();
			goal.pose.position.x = it->getX();
			goal.pose.position.y = it->getY();
			robot.goalPublisher.publish(goal);
			ros::Duration(0.1).sleep();
		}
		ROS_INFO("State: NAVIGATING");
	}
	return true;
}

bool endNavigation(tars_navigation::Navigation::Request  &req,
             tars_navigation::Navigation::Response &res) {
	
	if (robot.state == COMPUTING_RRT) {
		res.error = true;
		res.msg = "Not navigating";
	} else if (robot.state == INIT) {
		res.error = true;
		res.msg = "Not initiated";
	} else {
		robot.state = INIT;
		robot.goalReceived = false;
		robot.path.clear();
		robot.rrt.clear();
		res.error = false;
		res.msg = "OK";
		ROS_INFO("State: INIT");
	}
	return true;
}

/******************************/
/* FUNCIONES DE VISUALIZACION */
/******************************/

// Visualizacion del path
void publishPath(ros::Publisher& pub) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();
	marker.id = 0;
	marker.pose.orientation.x = 0;
	marker.pose.orientation.y = 0;		
	marker.pose.orientation.z = 0;		
	marker.pose.orientation.w = 1.0;				
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.scale.x = 0.05;
	marker.color.a = 1.0;
	marker.color.r = 0;
	marker.color.g = 0;
	marker.color.b = 1.0;	
	marker.action = visualization_msgs::Marker::ADD;	
	geometry_msgs::Point prev,next;
	prev.x = robot.init.getX();
	prev.y = robot.init.getY();
	prev.z = 0;
	next.z = 0;
	for (auto it = robot.path.begin();it!=robot.path.end();++it) {
		next.x = it->getX();
		next.y = it->getY();
		marker.points.push_back(prev);
		marker.points.push_back(next);
		prev = next;
	}
	pub.publish(marker);
}