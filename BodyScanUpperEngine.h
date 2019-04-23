#pragma once

using namespace std;
#include <process.h>
#include <windows.h>

#include "KeyPointsEstimator.h"
#include "Utils.h"
#include "Measures.h"
#include "MeasureThreads.h"

// Includes de PCL
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFile 


class BodyScanUpperEngine
{
private:
	///////////////////////////////////Variables///////////////////////////////////////////////////////////////////////////
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> user_model_cloud; // Nube que contiene el modelo 3D del usuario.
	KeyPointsEstimator * MyPointsEstimator; // Estimador de puntos claves para las medidas, sobre el modelo 3D del usuario.
	MeasuresMaker * MyMeasureMaker;
	Measure * measures; // Arreglo que contiene todas las medidas.
	float sensor_height; // Altura del sensor.
	/////////////////////////////////Funciones/////////////////////////////////////////////////////////////////////////////
	Measure findHeight(pcl::PointCloud<pcl::PointXYZRGBA> &cloud_host_, float _sensorHeight);
	Plane neck_plane;

public:
	
	BodyScanUpperEngine(string model_path, int model_orientation, std::vector<char> transforms);
	BodyScanUpperEngine::BodyScanUpperEngine(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> transformed_cloud);
	~BodyScanUpperEngine(void);
	inline Measure * GetAllMeasures(){return measures;}
	inline Measure GetHeightMeasure(){return measures[0];}
	inline Measure GetHeadtMeasure(){return measures[1];}
	inline Measure GetNeckMeasure(){return measures[2];}
	inline Measure GetShouldersMeasure(){return measures[3];}
	inline Measure GetChestMeasure(){return measures[4];}
	inline Measure GetwaistMeasure(){return measures[5];}
	inline float GetSensorHeight(){return sensor_height;}
	inline void SetSensorHeight(float _sensor_height){sensor_height = _sensor_height;}
	inline boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> GetUserCloud(){return user_model_cloud;}
	float GetCalfHeight(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> cloud,float crotch_height);
	float GetButtocksHeight(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> cloud, float crotch_height, float waist_height);
	Plane GetNeckPlane(){return neck_plane;}
	

	void UnpaintCloud();

	bool ProcessModel();
};

