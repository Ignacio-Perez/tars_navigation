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

#ifndef _RRT_HPP_
#define _RRT_HPP_

#include <list>
#include <cmath>
#include "kdtree.hpp"
#include "map.hpp"



class RRT
{
public:
	RRT() : edgeLength(0.2), rrtStar(true) {}

	void init(const utils::Vector2d& root, double edgeLength, bool rrtStar=true);
	void iterate();
	
	void clear() {
		tree.clear();
		parents.clear();
	}

	bool getPath(const utils::Vector2d& dst, std::list<utils::Vector2d>& path) const;

private:
	void iterateRRT();
	void iterateRRTStar();

	double edgeLength;
	bool rrtStar;
	utils::KdTree tree;
	std::vector<unsigned> parents;
	std::vector<double> cost;
	

};

inline
bool RRT::getPath(const utils::Vector2d& dst, std::list<utils::Vector2d>& path) const {
	if (tree.size()==0) {
		return false;
	}
	unsigned index = tree.getNearest(dst);
	if ( (dst - tree[index]).norm() > edgeLength) {
		return false;
	}
	path.clear();
	while (index != 0) {
		path.push_front(tree[index]);
		index = parents[index];	
	}
	return true;
}


inline
void RRT::init(const utils::Vector2d& root, double edgeLength, bool rrtStar) {
	tree.clear();
	parents.clear();
	cost.clear();
	cost.push_back(0);
	parents.push_back(0);
	tree.add(root);
	RRT::edgeLength = edgeLength;
	RRT::rrtStar = rrtStar;
}

inline
void RRT::iterate() {
	if (rrtStar) {
		iterateRRTStar();
	} else {
		iterateRRT();
	}
}


inline
void RRT::iterateRRT() {
	utils::Vector2d xrand;
	if (!MAP.sampleFree(xrand)) {
		return;
	}
	unsigned index = tree.getNearest(xrand);
	const utils::Vector2d& xnearest = tree[index];
	utils::Vector2d xnew = xnearest + edgeLength * (xrand-xnearest).normalized();
	if (MAP.isFree(xnearest,xnew)) {
		tree.add(xnew);
		parents.push_back(index);
	}

}

inline
void RRT::iterateRRTStar() {
	utils::Vector2d xrand;
	if (!MAP.sampleFree(xrand)) {
		return;
	}
	unsigned xnearestIndex = tree.getNearest(xrand);
	const utils::Vector2d& xnearest = tree[xnearestIndex];
	utils::Vector2d xnew = xnearest + edgeLength * (xrand-xnearest).normalized();
	if (MAP.isFree(xnearest,xnew)) {
		unsigned k = (unsigned)std::round(5.436563657 * std::log(tree.size()));
		std::vector<unsigned> indexes;
		tree.getKNearests(xnew,indexes,k);
		tree.add(xnew);
		unsigned xminIndex = xnearestIndex;
		double cmin = cost[xnearestIndex] + edgeLength;
		for (unsigned i=0;i<indexes.size();i++) {
			const utils::Vector2d& xnear = tree[indexes[i]];
			if (MAP.isFree(xnear,xnew) && cost[indexes[i]] + (xnear - xnew).norm() < cmin) {
				xminIndex = indexes[i];
				cmin = cost[indexes[i]] + (xnear - xnew).norm();
			}
		}
		parents.push_back(xminIndex);
		cost.push_back(cmin);
		for (unsigned i=0;i<indexes.size();i++) {
			const utils::Vector2d& xnear = tree[indexes[i]];
			if (MAP.isFree(xnew,xnear) && cmin + (xnew - xnear).norm() < cost[indexes[i]]) {
				parents[indexes[i]] = parents.size()-1;
			}
		}
	}
}

#endif