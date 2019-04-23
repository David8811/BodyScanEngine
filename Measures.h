#pragma once

#include <pcl/common/common.h>
#include "PlaneMaths.h"
#include "MyList.h"
#include "Utils.h"



class MeasuresMaker
{
private:
	pcl::PointXYZRGBA firstPoint;
	pcl::PointXYZRGBA * firstPointRef;
	PlaneMaths * MyPlaneMaths;
	bool AreValidChestPoints(MyList<pcl::PointXYZRGBA> result, float min_shouldersX, float max_shouldersX);
	Measure findPerimeter(pcl::PointCloud<pcl::PointXYZRGBA> &cloud, int r, int g, int b,int SIZE);
	Measure findPerimeter(MyList<pcl::PointXYZRGBA> result);

public:
	MeasuresMaker(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> model_cloud);
	~MeasuresMaker(void);
 	Measure GetMeasure ( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,Plane& resultPlane,float X=0,float Y=0, float Z=0,float shoulder_min = -1000,float shoulder_max = 1000,char measure_kind = 'n',float angle = 0,float epsilon = 0.005,float beta = 0.005);
};

