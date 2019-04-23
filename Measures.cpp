#include "Measures.h"
#include "PlaneMaths.h"
#include <pcl/visualization/cloud_viewer.h>

MeasuresMaker::MeasuresMaker(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> model_cloud)
{

	MyPlaneMaths = new PlaneMaths(model_cloud);

}


MeasuresMaker::~MeasuresMaker(void)
{
}

bool MeasuresMaker::AreValidChestPoints(MyList<pcl::PointXYZRGBA> result, float min_shouldersX, float max_shouldersX)
	{
	for(int i = 0; i < result.Size();i++)
					if(result.At(i)->x <= (min_shouldersX) || result.At(i)->x >= (max_shouldersX))
						return false;

	
	return true;

  }


Measure MeasuresMaker::findPerimeter(MyList<pcl::PointXYZRGBA> result)
{
	Measure meas;
	

    vector<CHPoint> pts = Utils::convexHull(result);
	float Perimeter = 0;

	for(int i = 0; i < pts.size()-1;i++)
		Perimeter += Utils::findDistance(*result.At(pts.at(i).index),*result.At(pts.at(i).index+1));

	meas.Value = Perimeter;

	return meas;
}

Measure MeasuresMaker::findPerimeter(pcl::PointCloud<pcl::PointXYZRGBA> &cloud, int r, int g, int b,int SIZE)
{
Measure meas;
		
		float Xmin = -1000;
		float Xmax = 1000;
		float Zmax = 1000;
		float Zmin = -1000;
	    

		int *p2;
		bool *p2_flags;
		int idx_rpts = 0;
		int p2_siz = SIZE;
		
		// Guardar índices de los puntos quese corresponden al lugar donde hacer la medición
		try
		{
		p2 = new int[p2_siz];
		p2_flags = new bool[p2_siz];
		int idx = 0;
		for(int i=0;i < cloud.size();i++){
			if (cloud.points[i].r == r && cloud.points[i].g == g && cloud.points[i].b == b){
			p2[idx] = i;
			p2_flags[idx++] = false;

		/*	cloud.points[i].r = 0;
			cloud.points[i].g = 0;
			cloud.points[i].b = 0;
			cloud.points[i].a = 0;*/

			}

		}
		}
		catch(...){}
		
		vector<CHPoint> pts = Utils::convexHull(cloud,p2,p2_siz);

		p2_siz = 0;
		
		for(vector<CHPoint>::iterator i = pts.begin();i != pts.end();i++){
			p2[p2_siz] = (*i).index;
			
			p2_siz++;
		}


		float Perimeter = 0;

		for(int i = 0;i < p2_siz-1;i++){
			
			Perimeter += Utils::findDistance(cloud.points[p2[i]],cloud.points[p2[i+1]]);

				 if(Xmin < cloud.points[p2[i]].x)
					 Xmin = cloud.points[p2[i]].x;

				 if(Xmax > cloud.points[p2[i]].x)
					 Xmax = cloud.points[p2[i]].x;

				 if(Zmin < cloud.points[p2[i]].z)
					 Zmin = cloud.points[p2[i]].z;

				 if(Zmax > cloud.points[p2[i]].z)
					 Zmax = cloud.points[p2[i]].z;

				 
		         meas.y = cloud.points[p2[i]].y;

		}
		
		float rx,rz;

		rx = (Xmin-Xmax);
		rz = (Zmin-Zmax);

		if(rx >= rz)
			meas.radius = rx;
		else
			meas.radius = rz;

		meas.Value = Perimeter;
		meas.x = Xmax + (Xmin-Xmax)/2;
		
		meas.z = Zmax + (Zmin-Zmax)/2;
		
		meas.xMax = Xmax;
		meas.xMin = Xmin;

		return meas;
}

Measure MeasuresMaker::GetMeasure( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, Plane& resultPlane,float X,float Y, float Z,float shoulder_min,float shoulder_max,char measure_kind,float angle,float epsilon,float beta)
{
	
	
	float x = X;
	float y = Y;
	float z = Z;
	pcl::RGB rgb ;
	Measure meas;
	meas.Value=meas.radius=meas.x=meas.y=meas.z = meas.xMax = meas.xMin = 0;
	
	rgb.r = 250;
	rgb.g = 0;
	rgb.b = 0;
	rgb.a = 128;

	if(measure_kind == 'm')
				{

					pcl::PointXYZRGBA myPoint;
					MyList<pcl::PointXYZRGBA> result;
					myPoint.x = X;
					myPoint.y = Y;
					myPoint.z = Z;

					result = MyPlaneMaths->ObtainingBodyPlaneByMeasurementPlane(myPoint,shoulder_min,shoulder_max,angle,resultPlane,cloud);

					
					cout << "Points at plane: " << result.Size() << endl;

					MyPlaneMaths->ChangeColor(result,rgb);

					meas = findPerimeter(*cloud,rgb.r,rgb.g,rgb.b,result.Size());

					return meas;
				}



	for(int i = 0; i < cloud->points.size() ; i++)
	{
		if ( cloud->points[i] .x >= x-0.008 && cloud->points[i] .x <= x+0.008)
			if ( cloud->points[i] .y >= y-0.008 && cloud->points[i] .y <= y+0.008)
		{
			
				firstPoint =  cloud->points[i] ;
				firstPointRef =  &cloud->points[i] ;
							
				/// Cambio de colores Borrar !!!!!! ///
					
		/*		cudaEvent_t start, stop;
				cudaEventCreate(&start);
				cudaEventCreate(&stop);
				cudaEventRecord( start, 0 );*/
															
			    auto result = MyPlaneMaths->ObtainingBodyPlanePointsUser1Point(firstPoint,cloud,resultPlane,angle,epsilon,beta);
				
		/*		cudaEventRecord( stop, 0 );
				cudaEventSynchronize( stop );
				float elapsedTime;
				cudaEventElapsedTime( &elapsedTime,start, stop );

//				cout << "Measure time: " << elapsedTime << endl;*/
				
				MyPlaneMaths->ChangeColor(result,rgb);

				
				if(measure_kind == 'c')
				{
					if(AreValidChestPoints(result,shoulder_min,shoulder_max))
					{
							meas = findPerimeter(*cloud,rgb.r,rgb.g,rgb.b,result.Size());
							meas.is_valid = true;
					}
					else
					{
					
							cout << "Not valid" << endl;
							meas.is_valid = false;
					}

					for(int i=0;i < cloud->size();i++)
						{
							cloud->points[i].r = 0;
							cloud->points[i].g = 0;
							cloud->points[i].b = 0;
							cloud->points[i].a = 0;
	
						}	

				}
				else
				{
				
				meas = findPerimeter(*cloud,rgb.r,rgb.g,rgb.b,result.Size());
		//		meas = findPerimeter(result);

/*				cudaEventRecord( stop, 0 );
				cudaEventSynchronize( stop );
				float elapsedTime;
				cudaEventElapsedTime( &elapsedTime,start, stop );*/

				
				}
			break;
		}
	}


	

	return meas;
}