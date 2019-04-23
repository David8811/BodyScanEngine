#pragma once
#include "Utils.h"
#include "Measures.h"
#include "Plane.h"

#include <pcl/io/io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

using namespace std;

class NeckMeasureThread
{
private:

  
  float Nxmin;
  float Nxmax;
  float NyHeightThs;
  float shoulder_min;
  float shoulder_max;
  char measure_kind;
  float angle;
  float epsilon;
  float beta;
  Measure temp;
  Measure neck;
  Plane neck_plane;
//  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> Mycloud;
  pcl::PointCloud<pcl::PointXYZRGBA> Mycloud;

public:
  string threadName;
  Measure GetNeck(){return neck;}
  Plane   GetNeckPlane(){return neck_plane;}

  NeckMeasureThread(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float _xmin= 0,float _xmax=0, float _yHeightTs=0,float shoulder_min = -1000,float shoulder_max = 1000,char measure_kind = 'n',float angle = 0,float epsilon = 0.005,float beta = 0.05)
  {
     
     Nxmin  = _xmin; 
	 Nxmax  = _xmax;
	 NyHeightThs  = _yHeightTs;
	 temp.Value = 1000;
	 pcl::copyPointCloud(*cloud,Mycloud);
	 
  }

  // In C++ you must employ a free (C) function or a static
  // class member function as the thread entry-point-function.
  // Furthermore, _beginthreadex() demands that the thread
  // entry function signature take a single (void*) and returned
  // an unsigned.
  static unsigned __stdcall ThreadStaticEntryPoint(void * pThis)
  {
      NeckMeasureThread * pthX = (NeckMeasureThread*)pThis;   // the tricky cast
      pthX->ThreadEntryPoint();           // now call the true entry-point-function

      // A thread terminates automatically if it completes execution,
      // or it can terminate itself with a call to _endthread().

      return 1;          // the thread exit code
  }

  void ThreadEntryPoint()
  {
    
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr A (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::copyPointCloud(Mycloud,*A);  
	  // This is the desired entry-point-function but to get
     // here we have to use a 2 step procedure involving
     // the ThreadStaticEntryPoint() function.

	
	Measure current;
	MeasuresMaker * MyMeasurer = new MeasuresMaker(A);

	for(float i = NyHeightThs+0.03;i>NyHeightThs-0.03;i-= 0.01)
		for(float j = 0;j < 45;j+=5)
		{

		  	
			current = MyMeasurer->GetMeasure(A,neck_plane,Nxmin+(Nxmax-Nxmin)/2,i,0,-1000,1000,'n',j,0.005,0.05);
			if(current.Value < temp.Value)
				temp = current;
			
		
		}

		neck = temp;
		
		
	  
    
  }
};

class ChestMeasureThread
{
private:

  
  float Cxmin;
  float Cxmax;
  float CyHeightThs;
  float Cshoulder_min;
  float Cshoulder_max;
  char measure_kind;
  float angle;
  float epsilon;
  float beta;
  bool first;
  Measure previous_chest,chest;
  Plane chest_plane;
//  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> Mycloud;
  pcl::PointCloud<pcl::PointXYZRGBA> Mycloud;
  
  

public:
  string threadName;
  
  Measure GetChest(){return chest;}
  Plane GetChestPlane(){return chest_plane;}
  
  ChestMeasureThread(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float _xmin= 0,float _xmax=0, float _yHeightTs=0,float _shoulder_min = -1000,float _shoulder_max = 1000,char measure_kind = 'n',float angle = 0,float epsilon = 0.005,float beta = 0.05)
  {
     
	/* pcl::PointCloud<pcl::PointXYZRGBA>* A = new pcl::PointCloud<pcl::PointXYZRGBA>();
	 Mycloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>>(A);*/

     Cxmin  = _xmin; 
	 Cxmax  = _xmax;
	 CyHeightThs  = _yHeightTs;
	 Cshoulder_min = _shoulder_min;
	 Cshoulder_max = _shoulder_max;
	 first = true;

	 pcl::copyPointCloud(*cloud,Mycloud);
	 
	  

  }

  // In C++ you must employ a free (C) function or a static
  // class member function as the thread entry-point-function.
  // Furthermore, _beginthreadex() demands that the thread
  // entry function signature take a single (void*) and returned
  // an unsigned.
  static unsigned __stdcall ChestThreadStaticEntryPoint(void * pThis)
  {
      ChestMeasureThread * pthX = (ChestMeasureThread*)pThis;   // the tricky cast
      pthX->ThreadEntryPoint();           // now call the true entry-point-function

      // A thread terminates automatically if it completes execution,
      // or it can terminate itself with a call to _endthread().

      return 1;          // the thread exit code
  }

  void ThreadEntryPoint()
  {
    
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr B (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::copyPointCloud(Mycloud,*B);  
	  // This is the desired entry-point-function but to get
     // here we have to use a 2 step procedure involving
     // the ThreadStaticEntryPoint() function.

	MeasuresMaker * MyMeasureMaker = new MeasuresMaker(B);

	
	while(true)
{



if(first)
chest = MyMeasureMaker->GetMeasure(B,chest_plane,Cxmin+(Cxmax-Cxmin)/2,CyHeightThs,0,Cshoulder_min,Cshoulder_max,'c',0,0.005,0.015);


if(first && chest.is_valid)
{
	first = false;
	while(true)
		{
			if(chest.is_valid && chest.Value != 0)
			{	
				previous_chest = chest;
				CyHeightThs-= 0.01;
				chest = MyMeasureMaker->GetMeasure(B,chest_plane,Cxmin+(Cxmax-Cxmin)/2,CyHeightThs,0,Cshoulder_min,Cshoulder_max,'c',0,0.005,0.015);
			}
			else
			{
				chest = previous_chest;
				break;
			}

		}
	
	
}
else
{
	if(!first)
	{
	chest = previous_chest;
	break;
	}
	else
	{
		while(true)
		{
			if(!chest.is_valid)
			{	
				CyHeightThs+= 0.01;
				chest = MyMeasureMaker->GetMeasure(B,chest_plane,Cxmin+(Cxmax-Cxmin)/2,CyHeightThs,0,Cshoulder_min,Cshoulder_max,'c',0,0.005,0.015);
					if(chest.is_valid)
						previous_chest = chest;
			}
			else
			{
				chest = previous_chest;
				break;
			}

		}
	   break;
		}
		}
	}
	

  }
};

class CrunchMeasureThread
{
private:

  
  float Cxmin;
  float Cxmax;
  float CyHeightThs;
  float Cshoulder_min;
  float Cshoulder_max;
  char measure_kind;
  float angle;
  float epsilon;
  float beta;
  bool first;
  Measure previous_crunch,crunch;
  Plane crunch_plane;
//  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> Mycloud;
  pcl::PointCloud<pcl::PointXYZRGBA> Mycloud;
  
  

public:
  string threadName;
  
  Measure GetCrunch(){return crunch;}
  Plane GetcrunchPlane(){return crunch_plane;}
  
  CrunchMeasureThread(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float _xmin= 0,float _xmax=0, float _yHeightTs=0,float _shoulder_min = -1000,float _shoulder_max = 1000,char measure_kind = 'n',float angle = 0,float epsilon = 0.005,float beta = 0.05)
  {
     
	/* pcl::PointCloud<pcl::PointXYZRGBA>* A = new pcl::PointCloud<pcl::PointXYZRGBA>();
	 Mycloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>>(A);*/

     Cxmin  = _xmin; 
	 Cxmax  = _xmax;
	 CyHeightThs  = _yHeightTs;
	 Cshoulder_min = _shoulder_min;
	 Cshoulder_max = _shoulder_max;
	 first = true;

	 pcl::copyPointCloud(*cloud,Mycloud);
	 
	  

  }

  // In C++ you must employ a free (C) function or a static
  // class member function as the thread entry-point-function.
  // Furthermore, _beginthreadex() demands that the thread
  // entry function signature take a single (void*) and returned
  // an unsigned.
  static unsigned __stdcall ThreadStaticEntryPoint(void * pThis)
  {
      CrunchMeasureThread * pthX = (CrunchMeasureThread*)pThis;   // the tricky cast
      pthX->ThreadEntryPoint();           // now call the true entry-point-function

      // A thread terminates automatically if it completes execution,
      // or it can terminate itself with a call to _endthread().

      return 1;          // the thread exit code
  }

  void ThreadEntryPoint()
  {
    
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr B (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::copyPointCloud(Mycloud,*B);  
	  // This is the desired entry-point-function but to get
     // here we have to use a 2 step procedure involving
     // the ThreadStaticEntryPoint() function.

	MeasuresMaker * MyMeasureMaker = new MeasuresMaker(B);

	

	

		while(true)
		{

			if(first)
			{
		     crunch = MyMeasureMaker->GetMeasure(B,crunch_plane,Cxmin+(Cxmax-Cxmin)/2,CyHeightThs,0,Cshoulder_min,Cshoulder_max,'c',0,0.003,0.02);
			 first = false;
			}

			if(!crunch.is_valid)
			{	
				CyHeightThs+= 0.005;
				crunch = MyMeasureMaker->GetMeasure(B,crunch_plane,Cxmin+(Cxmax-Cxmin)/2,CyHeightThs,0,Cshoulder_min,Cshoulder_max,'c',0,0.003,0.02);
					if(crunch.is_valid)
						previous_crunch = crunch;
			}
			else
			{
				crunch = previous_crunch;
				crunch.y = CyHeightThs;
				break;
			}

		}
	   
  }
};