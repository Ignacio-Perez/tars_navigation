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
#ifndef _MAP_HPP_
#define _MAP_HPP_

#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <cmath>

#include "random.hpp"
#include "vector2d.hpp"


class Map
{
public:
	~Map() {}
	Map(const Map& other) = delete;
	void operator=(const Map& other) = delete;
	static Map& getInstance() {static Map instance;return instance;}
	#define MAP Map::getInstance()
	void setData(const nav_msgs::OccupancyGrid::ConstPtr& msg,double inflateRadius);
	const nav_msgs::OccupancyGrid& getMap() const {return map;}
	bool sampleFree(utils::Vector2d& pos) const;
	bool isFree(const utils::Vector2d& pos) const;
	bool isFree(const utils::Vector2d& src, const utils::Vector2d& dst) const;
	bool isInitiated() const {return !freeCells.empty();}

private:
	Map();
	
	nav_msgs::OccupancyGrid map;
	unsigned seq;
	std::vector<unsigned> freeCells;

};

inline
Map::Map()
: seq (0) {}


inline 
bool Map::isFree(const utils::Vector2d& src, const utils::Vector2d& dst) const {
	if (src == dst) {
		return isFree(src);
	}
	if (!isFree(src) || !isFree(dst)) {
		return false;
	}
	utils::Vector2d u = dst;
	u -= src;
	double length = u.norm();
	u /= length;
	u *= map.info.resolution;
	utils::Vector2d x = src+u;
	double dist = map.info.resolution;
	while (dist < length) {
		if (!isFree(x)) {
			return false;
		}
		x += u;
		dist += map.info.resolution;
	}
	return true;
}

inline 
bool Map::sampleFree(utils::Vector2d& pos) const {
	if (freeCells.size()==0) {
		ROS_ERROR("Cannot sample free space");
		return false;
	}
	unsigned cell = freeCells[RANDOM(freeCells.size())];
	unsigned i = cell % map.info.width;
	unsigned j = cell / map.info.width;
	double x = i * map.info.resolution + RANDOM()*map.info.resolution;
	double y = j * map.info.resolution + RANDOM()*map.info.resolution;
	pos.set(x,y);
	return true;
}

inline
bool Map::isFree(const utils::Vector2d& pos) const {
	int i = (int)std::floor(pos.getX() / map.info.resolution);
	int j = (int)std::floor(pos.getY() / map.info.resolution);
	if (i<0 || i>= map.info.width || j<0 || j>= map.info.height) {
		return false;
	}
	return map.data[j*map.info.width+i]==0;
}


inline
void Map::setData(const nav_msgs::OccupancyGrid::ConstPtr& msg, double inflateRadius) {
	map.header = msg->header;
	map.info = msg->info;

	map.header.seq = seq++;
	map.header.stamp = ros::Time::now();
	map.info.map_load_time = ros::Time::now();
	
	map.data.assign(map.info.width * map.info.height, 0);
	freeCells.clear();

	unsigned offset = (unsigned)std::ceil(inflateRadius / map.info.resolution)+1;

	double x0 = map.info.resolution / 2;
	double y0 = map.info.resolution / 2;

	for (int j=0;j< map.info.height; j++) {
		for (int i= 0; i< map.info.width; i++) {

			signed char data = msg->data[j*map.info.width+i];
			if (data<0 || data > 50) {
				map.data[j*map.info.width+i] = 100;
				for (int a = j-offset; a < j+offset; a++) {
					if (a <0 || a>= map.info.height) {
						continue;
					}
					double y1 = a*map.info.resolution + map.info.resolution/2;
					for (int b = i-offset; b<i+offset; b++) {
						if (b <0 || b>= map.info.width) {
							continue;
						}
						double x1 = b*map.info.resolution + map.info.resolution / 2;
						double dist = std::sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));
						if (dist <= inflateRadius) {
							map.data[a*map.info.width+b] = 100;
						}
					}
				}
			}
			x0 += map.info.resolution;
		}
		x0 = map.info.resolution / 2;
		y0 += map.info.resolution;
	}
	for (unsigned i=0;i<map.data.size();i++) {
		if (map.data[i]==0) {
			freeCells.push_back(i);
		}
	}
	

}

#endif