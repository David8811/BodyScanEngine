#pragma once



using namespace std;

#include <iostream>

#include <vector>
#include <map>

// Includes de OpenCv
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>


// Includes de PCL
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include "KeyPoints2DEstimator.h"

const int IMAGE_WIDTH =  1024;
const int IMAGE_HEIGHT =  768;

 struct MeasurePoint
{
	float measure_xmin;
	float measure_xmax;
	float measure_ymin;
	float measure_ymax;

};

class KeyPointsEstimator
{
private:

	/// Variables para guardar las relaciones entre la nube proyectada y la sin proyectar.
    vector<int> idxs;  
    map<int,int> cloudIdxRel;

    
	/////////////////////////////////////////////////////////////////////////////////////////////////
	// Variables de PCL
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> model_cloud; //  "model_cloud", es la nube de puntos que contiene el modelo del usuario obtenido por el KinectFusion.
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> proyectable_cloud; // Contenedor para almacenar la nube model_cloud, en formato proyectable y oraganizado.
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> side_proyectable_cloud; // Contenedor para almacenar la nube model_cloud, en formato proyectable y oraganizado (vista lateral).
	//////////////////////////////////////////////////////////////////////////////////////////////////
	/// Funciones
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void makeProjectable(); // Convierte una nube no organizada y no proyectable, en una organizada y proyectable.
	void make_side_Projectable(); // Convierte una nube no organizada y no proyectable, en una organizada y proyectable. (vista lateral)
	void FilterCloud(float valor,int w, int h,int x, int y); // Rellena puntos en blanco de la nube. !! Reemplazar por el promedio de los puntos vecinos o no utilizar del todo.
	void FilterSideCloud(float valor,int w, int h,int x, int y); // Rellena puntos en blanco de la nube. !! Reemplazar por el promedio de los puntos vecinos o no utilizar del todo.
	MeasurePoint GetMeasureLocation(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> _proyectable_cloud,CvPoint a, CvPoint b);
	MeasurePoint GetHeadMeasureLocation(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> _proyectable_cloud,CvPoint neck_a,CvPoint neck_b,CvPoint head_point);
	MeasurePoint GetShouldersLocation(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> _proyectable_cloud,CvPoint a, CvPoint b);
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


public:
    KeyPointsEstimator(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> _model_cloud); //constructor
    ~KeyPointsEstimator();
	MeasurePoint * GetKeyPoints(); // Halla los puntos claves 2D del modelo del cuerpo. (Axilas,hombros,cuello,ect)
	
};

