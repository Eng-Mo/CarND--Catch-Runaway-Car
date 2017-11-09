/*
 * Tools.h
 *
 *  Created on: Oct 31, 2017
 *      Author: mo
 */
#include<math.h>

using namespace std;
struct Point{
	double x;
	double y;
	double radiaus;
};
#ifndef TOOLS_H_
#define TOOLS_H_

class Tools {
public:
	Tools();
	Point calcCenter(Point p1, Point p2, Point p3);
	virtual ~Tools();
};

#endif /* TOOLS_H_ */
