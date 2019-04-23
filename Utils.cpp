#include "Utils.h"


Utils::Utils(void)
{
}


Utils::~Utils(void)
{
}

 pcl::PointXYZRGBA* Utils::findLowestPoint(pcl::PointXYZRGBA* points, int count,CoordAxis axis){

	if (points == NULL || count <=0)
			return NULL;

		pcl::PointXYZRGBA *result = &points[0];
		if (axis == X_AXIS){
			for(int i =0;i < count;i++)
			{
				if (points[i].x < result->x)
					result = &points[i];
			}
		} else if (axis == Y_AXIS){
			for(int i =0;i < count;i++)
			{
				if (points[i].y < result->y)
					result = &points[i];
			}
		} else if (axis == Z_AXIS){
			for(int i =0;i < count;i++)
			{
				if (points[i].z < result->z)
					result = &points[i];
			}
		}

		return result;



}

 pcl::PointXYZRGBA* Utils::findHighestPoint(pcl::PointXYZRGBA* points, int count,CoordAxis axis)
	{
		if (points == NULL || count <=0)
			return NULL;

		pcl::PointXYZRGBA *result = &points[0];
		if (axis == X_AXIS)
		{
			for(int i =0;i < count;i++)
			{
				if (points[i].x > result->x)
					result = &points[i];
			}
		} else if (axis == Y_AXIS){
			for(int i =0;i < count;i++)
			{
				if (points[i].y > result->y)
					result = &points[i];
			}
		} else if (axis == Z_AXIS){
			for(int i =0;i < count;i++)
			{
				if (points[i].z > result->z)
					result = &points[i];
			}
		}

		return result;
	}

 pcl::PointXYZRGBA * Utils::cloudToArray(pcl::PointCloud<pcl::PointXYZRGBA> &cloud_host_){
	
		pcl::PointXYZRGBA *arr = new pcl::PointXYZRGBA[cloud_host_.size()];

		for(int i = 0;i < cloud_host_.size();i++){
		 arr[i] = cloud_host_[i];
	 }
		return arr;
	}

double Utils::findDistance(pcl::PointXYZRGBA& point1, pcl::PointXYZRGBA& point2)
{

return sqrt(pow(point1.x - point2.x,2) + pow(point1.y - point2.y,2) + pow(point1.z - point2.z,2));
}