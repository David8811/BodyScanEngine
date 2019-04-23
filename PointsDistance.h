#pragma once

#include <pcl/common/common.h>
#include "Utils.h"

class PointsDistance
{
public:
pcl::PointXYZRGBA * point1;
pcl::PointXYZRGBA * point2;
float Distance;

PointsDistance()
{

}

PointsDistance(pcl::PointXYZRGBA * ppoint1,pcl::PointXYZRGBA * ppoint2)
{
	point1 = ppoint1;
	point2 = ppoint2;
	Distance = Utils::PointDistance(point1,point2);
}

void SetPoints(pcl::PointXYZRGBA * ppoint1,pcl::PointXYZRGBA * ppoint2)
{
	point1 = ppoint1;
	point2 = ppoint2;
	Distance = Utils::PointDistance(point1,point2);
}

};