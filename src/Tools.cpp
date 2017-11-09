/*
 * Tools.cpp
 *
 *  Created on: Oct 31, 2017
 *      Author: mo
 */

#include "Tools.h"

Tools::Tools() {
	// TODO Auto-generated constructor stub

}

Tools::~Tools() {
	// TODO Auto-generated destructor stub
}

Point Tools::calcCenter(Point p1, Point p2, Point p3)
{
	Point center;
//	c->center.w = 1.0;
	Point v1 = p1;
	Point v2 = p2;
	Point v3 = p3;
	double bx = v1.x;   double by = v1.y;
	double cx = v2.x; double cy = v2.y;
	double dx = v3.x; double dy = v3.y;
	double temp = cx*cx+cy*cy;
	double bc = (bx*bx + by*by - temp)/2.0;
	double cd = (temp - dx*dx - dy*dy)/2.0;
	double det = (bx-cx)*(cy-dy)-(cx-dx)*(by-cy);
	if (fabs(det) < 1.0e-6) {
		center.x = center.y = 1.0;
//		c->center.w = 0.0;
//		c->v1 = *v1;
//		c->v2 = *v2;
//		c->v3 = *v3;
		return center;
		}
	det = 1/det;
	center.x = (bc*(cy-dy)-cd*(by-cy))*det;
	center.y = ((bx-cx)*cd-(cx-dx)*bc)*det;
	cx = center.x; cy = center.y;
	center.radiaus = sqrt((cx-bx)*(cx-bx)+(cy-by)*(cy-by));



	return center;

}

