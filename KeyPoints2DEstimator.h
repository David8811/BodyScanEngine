#pragma once

// Includes de OpenCv
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <pcl/common/common.h>


const int IS_DEBUGGING = 0;

class KeyPoints2DEstimator
{
private:
/// Variables de OpenCV///
    IplImage * model_img;  // Estructura utilzada para almacenar la imagen de OpenCv del modelo tridimensional
	IplImage * side_model_img;  // Estructura utilizada para almacenar la imagen de OpenCv del modelo tridimensional de la vista lateral.
	IplImage * model_img_cntr; // Imagen del contorno de la vista frontal
	CvMemStorage* storage;
	CvSeq* contour; // Contorno de la imagen frontal
	CvSeq * biggestcontour; // Almacenamiento del mayor contorno encontrado en la vista frontal
	
	CvPoint * key_points; // Almacenamiento de lo puntos claves encontrados
////Funciones/////////////////////////////////////////////////////////////////////////////
   void FillModelImage(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> proyectable_cloud); // Crea la imagen 2D a partir de la nube proyectable del modelo 3D
   void FillSideModelImage(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> proyectable_cloud); // Crea la imagen 2D a partir de la nube proyectable del modelo 3D (vista lateral)
   void FindChestPoints(CvPoint* pointArray); // Halla los puntos de las axilas key_points[0], key_points[1]
   void FindCrunchPoint(CvPoint* pointArray); // Halla el punto clave de la entrepierna key_points[9]
   void FindChestPoints2(int nomdef,CvConvexityDefect* defectArray); // Algoritmo redundante que se ejecuta de fallar el primero
   void FindShouldersPoints(CvPoint chest_point_a,CvPoint chest_point_b,CvPoint * pointAray); // Algoritmo para hallar los puntos de los hombros
   void FindWaistPoints(CvPoint left,CvPoint right,CvPoint * pointArray); // Algoritmo para hallar los puntos que definen el final del modelo, tomados como los de la cintura.
   void FindNeckPoints(CvPoint shoulder_left, CvPoint shoulder_right, CvConvexityDefect* defectArray, CvSeq* defect); // Algoritmo para hallar los puntos del cuello.
   void FindHeadPoint(CvPoint * pointArray);
   bool CheckForErrors(CvPoint * _key_points); // Chequea posibles errores en la información obtenida en los puntos claves. Devuelve verdadero si todo resultó correcto.
   void GetASMKeyPoints(IplImage * image);
////////////////////////////////////////////////////////////////////////////////////////

public:
	KeyPoints2DEstimator(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> proyectable_cloud,boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> side_proyectable_cloud); // Constructor de la clase
	~KeyPoints2DEstimator(void);
	CvPoint * Get2DKeyPoints();

};

