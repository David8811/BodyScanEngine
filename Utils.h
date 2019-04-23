#pragma once

#include <pcl/common/common.h>
#include "MyList.h"
#include "Plane.h"

using namespace std;

typedef float coord_t;         // coordinate type
typedef long double coord2_t;  // must be big enough to hold 2*max(|coordinate|)^2

struct Measure{
float x;
float y;
float z;
float radius;
Plane measure_plane;
float Value;
float xMin;
float xMax;

bool is_valid;
};


struct CHPoint {
	coord_t x, y;
	int index;

	bool operator <(const CHPoint &p) const {
		return x < p.x || (x == p.x && y < p.y);
	}
};

 class Utils
{

public:
	Utils(void);
	~Utils(void);
	static enum CoordAxis {X_AXIS,Y_AXIS,Z_AXIS};
	static pcl::PointXYZRGBA* findLowestPoint(pcl::PointXYZRGBA* points, int count,CoordAxis axis);
	static pcl::PointXYZRGBA* findHighestPoint(pcl::PointXYZRGBA* points, int count,CoordAxis axis);
	static pcl::PointXYZRGBA * cloudToArray(pcl::PointCloud<pcl::PointXYZRGBA> &cloud_host_);
	static double findDistance(pcl::PointXYZRGBA& point1, pcl::PointXYZRGBA& point2);

static	float PointDistance ( pcl::PointXYZRGBA p1 , pcl::PointXYZRGBA p2)
{
	return sqrt ( pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2) + pow((p1.z - p2.z),2) ) ;
}

static float PointDistance ( pcl::PointXYZRGBA * p1 , pcl::PointXYZRGBA * p2)
{
	return sqrt ( pow((p1->x - p2->x),2) + pow((p1->y - p2->y),2) + pow((p1->z - p2->z),2) ) ;
}
// esta es la distancia elevada al cuadrado 
static float PointDistancePower2 ( pcl::PointXYZRGBA * p1 , pcl::PointXYZRGBA * p2)
{
	  auto result = pow((p1->x - p2->x),2) + pow((p1->y - p2->y),2) + pow((p1->z - p2->z),2)  ;
	  return result;
}



static coord2_t cross(const CHPoint &O, const CHPoint &A, const CHPoint &B)
{
	return (A.x - O.x) * (coord2_t)(B.y - O.y) - (A.y - O.y) * (coord2_t)(B.x - O.x);
}
 
// Returns a list of points on the convex hull in counter-clockwise order.
// Note: the last point in the returned list is the same as the first one.
static vector<CHPoint> convex_hull(vector<CHPoint> P)
{
	int n = P.size(), k = 0;
	vector<CHPoint> H(2*n);
 
	// Sort points lexicographically
	sort(P.begin(), P.end());
 
	// Build lower hull
	for (int i = 0; i < n; i++) {
		while (k >= 2 && cross(H[k-2], H[k-1], P[i]) <= 0) k--;
		H[k++] = P[i];
	}
 
	// Build upper hull
	for (int i = n-2, t = k+1; i >= 0; i--) {
		while (k >= t && cross(H[k-2], H[k-1], P[i]) <= 0) k--;
		H[k++] = P[i];
	}
 
	H.resize(k);
	return H;
}

static vector<CHPoint> convexHull(MyList<pcl::PointXYZRGBA> result){

	vector<CHPoint> pts;


	for(int i = result.Size()-1;i >= 0;i--){
		CHPoint p;
		p.x = result.At(i)->x;
		p.y = result.At(i)->z;
		p.index = i;
		pts.push_back(p);
	}
	
	pts = convex_hull(pts);

	return pts;

}

static vector<CHPoint> convexHull(pcl::PointCloud<pcl::PointXYZRGBA> &cloud_host_,int *p2,int p2_siz){

	vector<CHPoint> pts;


	for(int i = 0;i < p2_siz;i++){
		CHPoint p;
		p.x = cloud_host_.points[p2[i]].x;
		p.y = cloud_host_.points[p2[i]].z;
		cloud_host_.points[p2[i]].r=0;
		p.index = p2[i];
		pts.push_back(p);
	}
	
	pts = convex_hull(pts);

	return pts;

}
	
};

