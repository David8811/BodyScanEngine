

#include "KeyPoints2DEstimator.h"
#include "KeyPointsEstimator.h"

#include "AAM_IC.h"
#include "AAM_Basic.h"
#include "AAM_MovieAVI.h"
#include "VJfacedetect.h"

#include <windows.h>
#include <string>

KeyPoints2DEstimator::KeyPoints2DEstimator(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> proyectable_cloud,boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> side_proyectable_cloud)
{
	// Inicializando variables
	key_points = new CvPoint[10];
	model_img = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_8U, 1);
	side_model_img = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_8U, 1);
	model_img_cntr = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_8U, 3); // Imagen del contorno de la vista frontal
	storage = cvCreateMemStorage(0);
	contour = 0; // Contorno de la imagen frontal
	biggestcontour= 0;

	FillModelImage(proyectable_cloud);
	FillSideModelImage(side_proyectable_cloud);
	
    char buffer[MAX_PATH];
	char tick_buffer[32];
    GetModuleFileName( NULL, buffer, MAX_PATH );
    string::size_type pos = string( buffer ).find_last_of( "\\/" );
	string current_directory_path = string( buffer ).substr( 0, pos); 


	DWORD tick_count = GetTickCount();

	ltoa((long)tick_count, tick_buffer, 10);

	string tickstr = string(tick_buffer);

	cvSaveImage((current_directory_path + "/" + "Silhouette " +  tickstr + ".jpg").c_str(),model_img);

//	GetASMKeyPoints(model_img);

	if(IS_DEBUGGING)
	{
	
		cvShowImage("ModelImage",model_img);
		cvShowImage("SideModelImage",side_model_img);
		cvWaitKey(0);
	
	
	}

}


void KeyPoints2DEstimator::GetASMKeyPoints(IplImage * image)
{
	
    AAM_Pyramid model;

	char buffer[MAX_PATH];
    GetModuleFileName( NULL, buffer, MAX_PATH );
    string::size_type pos = string( buffer ).find_last_of( "\\/" );
	string current_directory_path = string( buffer ).substr( 0, pos);


	model.ReadModel(current_directory_path+"\\body_landmarks.amf");
	VJfacedetect facedet;
	

	IplImage* drawing_image = cvCreateImage(cvSize(image->width,image->height),8,3);
		AAM_Shape Shape;

		cvCvtColor(image,drawing_image,CV_GRAY2RGB);
	
		bool flag = flag = model.InitShapeFromDetBox(Shape, facedet, image);
		if(flag == false) {
			fprintf(stderr, "The image doesn't contain any faces\n");
			exit(0);
		}

		
		model.Fit(image, Shape, 500, false);		

		model.Draw(drawing_image, Shape, 2);
		
		cvNamedWindow("Fitting");
		cvShowImage("Fitting", drawing_image);
		cvWaitKey(0);

		cvReleaseImage(&drawing_image);
				
}

KeyPoints2DEstimator::~KeyPoints2DEstimator(void)
{
}

void KeyPoints2DEstimator::FillModelImage(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> proyectable_cloud)
{
											
	for(int i = 0; i < IMAGE_WIDTH;i++)
		for(int j = 0; j < IMAGE_HEIGHT;j++)
		{
		int index = j*IMAGE_WIDTH+i;

		if(proyectable_cloud->points[index].z != 0)
			cvSet2D(model_img,j,i,cvScalar(255));
		else
			cvSet2D(model_img,j,i,cvScalar(0));
				
		}
			

}

void KeyPoints2DEstimator::FillSideModelImage(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> proyectable_cloud)
{
											
	for(int i = 0; i < IMAGE_WIDTH;i++)
		for(int j = 0; j < IMAGE_HEIGHT;j++)
		{
		int index = j*IMAGE_WIDTH+i;

		if(proyectable_cloud->points[index].z != 0)
			cvSet2D(side_model_img,j,i,cvScalar(255));
		else
			cvSet2D(side_model_img,j,i,cvScalar(0));
				
		}
			

}

// Partiendo de que el usuario está en la pose indicada, se buscan los  puntos del contorno
// que se corresponden con las axilas, teniendo en cuenta el ángulo que forman los brazos con el cuerpo
// en dicha postura.
void KeyPoints2DEstimator::FindChestPoints(CvPoint* pointArray)
{

	
	 
	 int lower = 0;
	 int higger = 100000;
	 int body_height;
	 CvPoint P;
	 
	 // Hallar punto más bajo y punto más alto

	 for(int i = 0; i < biggestcontour->total;i++)
	 {
		 P = pointArray[i];
		// cvDrawCircle(imgcntr,P,5,cvScalar(255,0,0));

		 if(pointArray[i].y > lower)
		 lower = pointArray[i].y;
	     
		 if(pointArray[i].y < higger)
			 higger = pointArray[i].y;
	 }

	 body_height = lower - higger;

	 // Pintar puntos del contorno
	 CvPoint P1;
	 P1.x = -1;
	 P1.y = -1;

	 CvPoint P2;
	 P2.x = -1;
	 P2.y = -1;

	 CvPoint P3;
	 P3.x = -1;
	 P3.y = -1;

	 bool deepest_sel = true;

	 vector<CvPoint> posible_axilas;

	 for(int i = 0; i < biggestcontour->total-2;i++)
	 {
        
	    double angle = 0;
	    double d12, d32;
		 

        P1.x = pointArray[i].x;
		P2.x = pointArray[i+1].x;
		P3.x = pointArray[i+2].x;

		P1.y = pointArray[i].y;
		P2.y = pointArray[i+1].y;
		P3.y = pointArray[i+2].y;

		d12 = sqrt(pow(double(P1.x-P2.x),2)+pow(double(P1.y-P2.y),2));
		d32 = sqrt(pow(double(P3.x-P2.x),2)+pow(double(P3.y-P2.y),2));
		angle = abs(atan2((double)(P3.y-P2.y),double(P3.x-P2.x)) - atan2(double(P1.y-P2.y),double(P1.x-P2.x)));

		if(P1.y > P2.y && P3.y > P2.y)						
				{
					if(angle >=  -45*3.1415/180 && angle <= 115*3.1415/180)						
						{
							posible_axilas.push_back(pointArray[i+1]);
						}
		
				}
	 
	 }

	 auto iter = posible_axilas.begin();

	 for(int i = 0; i < posible_axilas.size();i++)
		 if(posible_axilas[i].y > IMAGE_HEIGHT/2)
			 posible_axilas.erase(iter+i);

	 if(posible_axilas.size() == 2)
	 {
		 key_points[0] = posible_axilas[0];
		 key_points[1] = posible_axilas[1];
	 }

	 cvDrawCircle(model_img,key_points[0],5,cvScalar(255,0,0),3);
	 cvDrawCircle(model_img,key_points[1],5,cvScalar(255,0,0),3);

	 if(IS_DEBUGGING)
	 {
	 cvShowImage("Axilas",model_img);
	 cvWaitKey(0);
	 }

}


/// Similar a la función para encontrar los puntos de las axilas, esta se centra en buscar la entrepierna

void KeyPoints2DEstimator::FindCrunchPoint(CvPoint * pointArray)
{
	 
	 // Pintar puntos del contorno
	 CvPoint P1;
	 P1.x = -1;
	 P1.y = -1;

	 CvPoint P2;
	 P2.x = -1;
	 P2.y = -1;

	 CvPoint P3;
	 P3.x = -1;
	 P3.y = -1;
	
	 vector<CvPoint> posible_crunch;

	 for(int i = 0; i < biggestcontour->total-2;i++)
	 {
        
	    double angle = 0;
	    double d12, d32;
		 

        P1.x = pointArray[i].x;
		P2.x = pointArray[i+1].x;
		P3.x = pointArray[i+2].x;

		P1.y = pointArray[i].y;
		P2.y = pointArray[i+1].y;
		P3.y = pointArray[i+2].y;

		d12 = sqrt(pow(double(P1.x-P2.x),2)+pow(double(P1.y-P2.y),2));
		d32 = sqrt(pow(double(P3.x-P2.x),2)+pow(double(P3.y-P2.y),2));
		angle = abs(atan2((double)(P3.y-P2.y),double(P3.x-P2.x)) - atan2(double(P1.y-P2.y),double(P1.x-P2.x)));

		if(P1.y > P2.y && P3.y > P2.y)					
			if(angle >=  -5*3.1415/180 && angle <= 75*3.1415/180)								
				posible_crunch.push_back(pointArray[i+1]);						
		
		
	 
	 }
	
	 CvPoint lower;
	 lower.y = 0;

	 for(int i = 0; i < posible_crunch.size();i++)
		 if(posible_crunch[i].y > lower.y)
		 {
		    key_points[9] = posible_crunch[i];
			lower.y = posible_crunch[i].y;
		 }
		 	 
	 cvDrawCircle(model_img,key_points[9],5,cvScalar(255,0,0),3);

	 if(IS_DEBUGGING)
	 {
	 cvShowImage("Entrepierna",model_img);
	 cvWaitKey(0);
	 }

}

// Este algoritmo utiliza los "defects" para hallar los puntos de las axilas, asumiendo que estos se encuentran en los lugares donde los 
// "defects" son más profundos
void KeyPoints2DEstimator::FindChestPoints2(int nomdef,CvConvexityDefect* defectArray)
{
	float Result = 0;
	float Result2 = 0;

	// Buscar los dos mayores "defects" de la secuencia
{
    for(int i = 0; i < nomdef; i++)
    {
		Result = defectArray[i].depth;

			if( Result > Result2)
             {
                 Result2 = Result;
				 key_points[0] = *defectArray[i].depth_point;	
             }
		
    }

	Result = 0, Result2 = 100000;

	for(int i = 0; i < nomdef; i++)
    {
		Result = abs(defectArray[i].depth_point->y - key_points[0].y);

		if(Result < Result2 && defectArray[i].depth_point->x != key_points[0].x)
             {
				 key_points[1] =   *defectArray[i].depth_point;
			     Result2 = Result;
             } 
    }
	}
}

// Igualmente basado en la postura del usuario, este algoritmo asume que los puntosmás xternos, a una distancia determinada de los puntos
// del pecho, con nu ángulo dado, se corresponden a los puntos de los hombros.
void KeyPoints2DEstimator::FindShouldersPoints(CvPoint chest_point_a,CvPoint chest_point_b,CvPoint * pointArray)
{
	CvPoint left, right;

	// Determinar cuál es el punto del pecho a la derecha, y cuál está a la izquierda
	if(chest_point_a.x > chest_point_b.x)
	{
		right = chest_point_a;
		left = chest_point_b;
	}
	else
	{
		right = chest_point_b;
		left = chest_point_a; 
	
	}
	
	float distance = abs(right.x - left.x);

	// Hallando los puntos de los hombros (izquierdo)

	float Result = 0;
	float Result2 = 1000000;
	
	for(int i = 0; i < biggestcontour->total; i++)
    {
	   float distance_s = abs(pointArray[i].y - right.y);
		double angle = atan2((double)-1*(pointArray[i].y-right.y),double(pointArray[i].x-right.x));  // -1 pq están invertidos los ejes en las Ys

		

		if(angle < 0)
			angle = 6.28+angle;

		angle = angle*180/3.1415;

		Result = angle;

		if(Result < Result2 && pointArray[i].x > right.x && pointArray[i].y < right.y)
             {
				if(distance_s > distance*5/100)
				 if(angle >= 10 && angle <= 100)
				 {
				 key_points[2] =   pointArray[i];
			     Result2 = Result;
				 }
             }      
    }

	// Hallando los puntos de los hombros (derecho)
	Result = 0, Result2 = 0;

	for(int i = 0; i < biggestcontour->total; i++)
    {
	    
		float distance_s = abs(pointArray[i].y - left.y);
		double angle = atan2((double)-1*(pointArray[i].y-left.y),double(pointArray[i].x-left.x)); // -1 pq están invertidos los ejes en las Ys

		if(angle < 0)
			angle = 6.28+angle;

		angle = angle*180/3.1415;

		Result = angle;

		if(Result > Result2 && pointArray[i].x < left.x && pointArray[i].y < left.y)
             {
				if(distance_s > distance*5/100)
				 if(angle >= 70 && angle <= 170)
				 {
				 key_points[3] =   pointArray[i];
			     Result2 = Result;
				 }
             }       
    }
}

// Halla los puntos de la cintura. Asuminedo que sean los puntos más bajos por adentro del límite marcado por los hombros
// left - hombro izquierdo, right - hombro derecho
void KeyPoints2DEstimator::FindWaistPoints(CvPoint _left,CvPoint _right,CvPoint * pointArray)
{
	CvPoint left, right;

	if(_left.x > _right.x)
	{
		right = _left;
		left = _right;
	}
	else
	{
		right = _right;
		left = _left; 
	
	}

	key_points[4] = pointArray[0];

	for(int i = 0; i < biggestcontour->total; i++)
    {
		float distX = abs(pointArray[i].x - left.x);
		
		if(pointArray[i].y > left.y && distX < abs(left.x - key_points[4].x))	 
				 key_points[4] =   pointArray[i];
			    
             
    }

	key_points[5] = pointArray[0];

	for(int i = 0; i < biggestcontour->total; i++)
    {
		float distX = abs(pointArray[i].x - right.x);

		if(pointArray[i].y > right.y && distX < abs(right.x - key_points[5].x))
				 key_points[5] =   pointArray[i];             
    }
}

void KeyPoints2DEstimator::FindNeckPoints(CvPoint shoulder_left,CvPoint shoulder_right,CvConvexityDefect* defectArray,CvSeq* defect)
{

 key_points[6] = *defectArray[0].depth_point;
 float Result = 10000;
 float Result2 = 10000;

	for(int i = 0; i < defect->total; i++)
    {
		Result = sqrt(pow(float(defectArray[i].depth_point->y-shoulder_left.y),2)+pow(float(defectArray[i].depth_point->x-shoulder_left.x),2));

		if(defectArray[i].depth_point->y < shoulder_left.y && Result < Result2)
             {
				 key_points[6] =   *defectArray[i].depth_point;
				 Result2 = Result;
			    
             }    
    }

	key_points[7] = *defectArray[0].depth_point;

	Result = 10000, Result2 = 10000;

	for(int i = 0; i < defect->total; i++)
    {
		if(defectArray[i].depth_point->y < shoulder_right.y)
		{
			Result = sqrt(pow(float(defectArray[i].depth_point->y-shoulder_right.y),2)+pow(float(defectArray[i].depth_point->x-shoulder_right.x),2));

			if(Result < Result2)
             {
				 key_points[7] =   *defectArray[i].depth_point;
			     Result2 = Result;
             } 
		}
	}
    free(defectArray);

}

void KeyPoints2DEstimator::FindHeadPoint(CvPoint * pointArray)
{
	key_points[8] = pointArray[0];

	for(int i = 0; i < biggestcontour->total; i++)		
		if(pointArray[i].y < key_points[8].y)
				 key_points[8] = pointArray[i]; 
             
    


}

bool KeyPoints2DEstimator::CheckForErrors(CvPoint * _key_points)
{

	// Chequea si los puntos claves más importantes tienen valores
if(_key_points[0].x >= 0 && _key_points[1].x >= 0 && _key_points[2].x >= 0 && _key_points[3].x >= 0)
	 {
		 if(abs(_key_points[0].y - _key_points[1].y) >= IMAGE_HEIGHT*10/100) // Chequea si los puntos de las axilas no están a aproximadamente la misma altura
		 {
		 cout << "*E04";
		 return false;
		 }
		 
		 if(abs(_key_points[2].y - _key_points[3].y) >= IMAGE_HEIGHT*10/100) // Chequea si los puntos de los hombros no están a aproximadamente la misma altura
		 {
		 cout << "*E05";
		 return false;
		 }
		 if(abs(_key_points[6].y - _key_points[7].y) >= IMAGE_HEIGHT*10/100) // Chequea si los puntos del cuello no están a aproximadamente la misma altura
		 {
			return false;
			cout << "*E06";
		 }

	 }
else
{
	cout << "*E03";
	return false;
}
    return true;
}

CvPoint * KeyPoints2DEstimator::Get2DKeyPoints()
{
	
	

	// puntero al primer contorno de la secuencia de contornos presente en la imagen que representa al modelo 3D del usuario
		cvFindContours(model_img, storage, &contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
		
	// Aproximando dicho contorno por un polígono
		contour = cvApproxPoly(contour, sizeof(CvContour), storage, CV_POLY_APPROX_DP,10, 1);
     	

     float Result = 0, Result2 = 0;

	 // Hallando el contorno de mayor área de la secuencia, el cual se debe corresponder con la figura del cuerpo (frontal)
     // Este procedimiento es necesario para eliminar ruidos indeseados en la imagen.
	 for(;contour;contour = contour->h_next)
	 {
		 Result = fabs( cvContourArea(contour, CV_WHOLE_SEQ) );
		 
		 if( Result > Result2)
             {
                 Result2 = Result;
				 biggestcontour = contour;
				
             } 
	 }

	 // Si se ha hallado un biggest contour quiered decir que se encontró una figura que presumiblemente es el usuario
     if(biggestcontour)
	 {
     
     // Convirtiendo el contorno en un arreglo de puntos
     CvPoint* pointArray = (CvPoint*)malloc(sizeof(CvPoint)*biggestcontour->total);
     cvCvtSeqToArray(biggestcontour, pointArray, CV_WHOLE_SEQ);
	  
	 for(int i  =0; i < biggestcontour->total;i++)
				cvDrawCircle(model_img,pointArray[i],5,cvScalar(255,0,0));
	 
	 if(IS_DEBUGGING)
	 {
	 cvShowImage("All points",model_img);
	 cvWaitKey(0);
	 }

	 // Hallando convex hull	 
	 CvSeq* hull = cvConvexHull2(biggestcontour, 0, CV_CLOCKWISE, 0);

	 // Hallando la concavidad del contorno para encontrar los ptos debajo de los brazos
	 CvSeq* defect = cvConvexityDefects(biggestcontour, hull, NULL);

	 // Calculando el número de "defects"
	 int nomdef = defect->total;

	 // Convirtiendo a array de "defects"
     CvConvexityDefect* defectArray = (CvConvexityDefect*)malloc(sizeof(CvConvexityDefect)*nomdef);
     cvCvtSeqToArray(defect, defectArray, CV_WHOLE_SEQ);

	 // Hallando el punto de la entrepierna
	 FindCrunchPoint(pointArray);

	 // Hallando los puntos de las axilas, quedan almacenados en key_points[0], key_points[1]
	 FindChestPoints(pointArray);
	 
		// Si ha fallado el primer método de encontrar los puntos de las axilas,
		// utilizamos el segundo basado en el convexity defects.
		if(key_points[0].x < 0 || key_points[1].x < 0)
			FindChestPoints2(nomdef,defectArray);
			

		// Hallando los puntos de los hombros, quedan almacenados en key_points[2], key_points[3]
		FindShouldersPoints(key_points[0],key_points[1],pointArray);

		// Hallando los puntos de la cintura, almacenados en key_points[4], key_points[5]
		FindWaistPoints(key_points[0],key_points[1],pointArray);

		//Hallando los puntos del cuello
		FindNeckPoints(key_points[2],key_points[3],defectArray,defect);

		//Hallando el punto más alto de la cabeza
		FindHeadPoint(pointArray);  // key_points[8] - punto de la cima de la cabeza.

		if(CheckForErrors(key_points))
		{
			cout << "Puntos claves bidimensionales encontrados correctamente" << endl;

			// Solo con propósitos de debuggeo mostrar la ubicación de los puntos claves encontrados
			for(int i  =0; i < 10;i++)
				cvDrawCircle(model_img,cvPoint(key_points[i].x,key_points[i].y),5,cvScalar(255,0,0),5,5);

			if(IS_DEBUGGING)
			{
			cvShowImage("Model Image",model_img);
			cvWaitKey(0);
			}

			return key_points;
		}
		else
		{
			cout << "Error en KeyPoints2DEstimator" << endl;
			key_points = NULL;
		}
		
	 }

	 key_points = NULL; // Si no se encontraron correctamente los puntos key_points se hace NULL;

	return key_points;
}