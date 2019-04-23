#pragma once

#include <pcl/common/common.h>

class Vector
{
public:
	Vector(void)
{
	X = 0.0f;
	Y = 0.0f;
	Z = 0.0f; 
}


~Vector(void)
{
}

float CartesianProduct(pcl::PointXYZRGBA point1,pcl::PointXYZRGBA point2)
{	
	return X*(point1.x - point2.x) + Y*(point1.y - point2.y) +Z*(point1.z - point2.z) ;
}

	
	double X;
	double Y;
	double Z;
};

