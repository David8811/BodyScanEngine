#pragma once

#include <pcl/common/common.h>
#include <math.h>
#include "Vector.h"

class Plane
{

	public:
	/// Normal al plano
	Vector Normal;
	// Punto perteneciente al plano
	pcl::PointXYZRGBA ReferencePoint;

	Plane(void)
	{

	}

	~Plane(void)
	{
	}

	bool Contains(pcl::PointXYZRGBA point,float epsilon)
	{
		return abs(Normal.CartesianProduct(ReferencePoint , point ) ) < epsilon ;
	}

};