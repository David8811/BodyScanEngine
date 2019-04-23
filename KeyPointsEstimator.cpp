

#include "KeyPointsEstimator.h"
#include "Utils.h"


/// Constructor de la clase. Crea los recursos necesarios, a partir de la información suministrada en la nube de puntos
/// "model_cloud", la cual contiene el modelo del usuario obtenido por el KinectFusion.
KeyPointsEstimator::KeyPointsEstimator(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> _model_cloud):model_cloud(new (pcl::PointCloud<pcl::PointXYZRGBA>)), 
																						      proyectable_cloud(new (pcl::PointCloud<pcl::PointXYZRGBA>)),side_proyectable_cloud(new (pcl::PointCloud<pcl::PointXYZRGBA>))
{
	try
	{
		pcl::copyPointCloud(*_model_cloud, *model_cloud); // Copia el modelo del usuario a model_cloud
		
	}
	catch(...)
	{
	// !!! MANEJO DE EXCEPCIONES PENDIENTE
		
	}
	
	proyectable_cloud->width = IMAGE_WIDTH;
	proyectable_cloud->height = IMAGE_HEIGHT;
	proyectable_cloud->resize(proyectable_cloud->width*proyectable_cloud->height);
	side_proyectable_cloud->resize(proyectable_cloud->size());

	cout << "Variables de PCL  inicializadas correctamente..." << endl; // Con propósitos de debuggeo. Quitar luego.

}

// Destructor de la clase
KeyPointsEstimator::~KeyPointsEstimator(void)
{
}


// Convierte la nube "unproyectable_cloud" en proyectable
void KeyPointsEstimator::makeProjectable()
{

for(int i =0;i < model_cloud->points.size();i++)
		{
	
			int x = IMAGE_WIDTH/2 + model_cloud->points[i].x/model_cloud->points[i].z*IMAGE_WIDTH;
			int y = IMAGE_HEIGHT/2 + model_cloud->points[i].y/model_cloud->points[i].z*IMAGE_HEIGHT;
			
			if (x < 0) x = 0;
			if (y < 0) y = 0;
			
			int index = IMAGE_WIDTH*y+x;

			if(index >= 0 && index < IMAGE_WIDTH*IMAGE_HEIGHT)
			{
				proyectable_cloud->points[index].x = model_cloud->points[i].x;
				proyectable_cloud->points[index].y = model_cloud->points[i].y;
				proyectable_cloud->points[index].z = model_cloud->points[i].z;

				cloudIdxRel[index] = i;

			if(model_cloud->points[i].z != 0)
				FilterCloud(model_cloud->points[i].z,2,2,x,y);
			}
		
		}

}

// Convierte la nube a proyectable para la vista lateral
void KeyPointsEstimator::make_side_Projectable()
{
	float farest = Utils::findHighestPoint(Utils::cloudToArray(*model_cloud),model_cloud->size(),Utils::CoordAxis::X_AXIS)->x;
	float nearest = Utils::findLowestPoint(Utils::cloudToArray(*model_cloud),model_cloud->size(),Utils::CoordAxis::X_AXIS)->x;


	for(int i =0;i < model_cloud->points.size();i++)
		{
			if(model_cloud->points[i].x > ((farest+nearest)/2))
			{
	
				int x = IMAGE_WIDTH/2 + model_cloud->points[i].z*IMAGE_WIDTH;
				int y = IMAGE_HEIGHT/2 + model_cloud->points[i].y/model_cloud->points[i].z*IMAGE_HEIGHT;
			
				if (x < 0) x = 0;
				if (y < 0) y = 0;
			
				int index = IMAGE_WIDTH*y+x;

					if(index >= 0 && index < IMAGE_WIDTH*IMAGE_HEIGHT)
					{
						side_proyectable_cloud->points[index].x = model_cloud->points[i].x;
						side_proyectable_cloud->points[index].y = model_cloud->points[i].y;
						side_proyectable_cloud->points[index].z = model_cloud->points[i].z;

				//	cloudIdxRel[index] = i;

					//	if(model_cloud->points[i].z != 0)
						//	FilterSideCloud(model_cloud->points[i].z,2,2,x,y);
					}
			}
		}


}
/// valor de profundidad a aplicar en esta zona.
/// w y h, lados del rectángulo de rellenado
//  centro del rectángulo de rellenado.
//  
void KeyPointsEstimator::FilterCloud(float valor,int w, int h,int x, int y)
{

	for(int i = x-w; i < x+w;i++)
	for(int j = y-h; j < y+h;j++)
	{
	int index = IMAGE_WIDTH*j+i;
	if(index >= 0 && index < IMAGE_WIDTH*IMAGE_HEIGHT)
		proyectable_cloud->points[index].z = valor;
	
	
	}

	
}

void KeyPointsEstimator::FilterSideCloud(float valor,int w, int h,int x, int y)
{

	for(int i = x-w; i < x+w;i++)
	for(int j = y-h; j < y+h;j++)
	{
	int index = IMAGE_WIDTH*j+i;
	if(index >= 0 && index < IMAGE_WIDTH*IMAGE_HEIGHT)
		side_proyectable_cloud->points[index].z = valor;
	
	
	}

	
}

MeasurePoint KeyPointsEstimator::GetMeasureLocation(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> _proyectable_cloud,CvPoint a, CvPoint b)
{

	MeasurePoint location;

float yFaceHeight = MIN(a.y,b.y) + (MAX(a.y,b.y) - MIN(a.y,b.y))/2;
	float xFaceHeightMi = MIN(a.x,b.x),xFaceHeightMa = MAX(a.x,b.x);	 

	float yFaceHeightMin = 0,yFaceHeightMax = 0,xFaceHeightMax = 0,xFaceHeightMin = 0,zFaceHeightMin = 0,zFaceHeightMax = 0;
	
	 
	for(int i = xFaceHeightMi; i <= xFaceHeightMa;i++)
	for(int j =  yFaceHeight-5;j <= yFaceHeight+5; j++)
	 {
		 int index  = j*IMAGE_WIDTH+i;
		 idxs.push_back(index);

	     if(index < _proyectable_cloud->size())
		 if (_proyectable_cloud->points[index].y != 0 && _proyectable_cloud->points[index].x != 0){
			 if ( yFaceHeightMin == 0 || yFaceHeightMin > _proyectable_cloud->points[index].y)
				 yFaceHeightMin = _proyectable_cloud->points[index].y;
			 if (yFaceHeightMax == 0 || yFaceHeightMax < _proyectable_cloud->points[index].y)
				 yFaceHeightMax = _proyectable_cloud->points[index].y;
			 if ( xFaceHeightMin == 0 || xFaceHeightMin > _proyectable_cloud->points[index].x)
				 xFaceHeightMin = _proyectable_cloud->points[index].x;
			 if (xFaceHeightMax == 0 || xFaceHeightMax < _proyectable_cloud->points[index].x)
				 xFaceHeightMax = _proyectable_cloud->points[index].x;
			 if ( zFaceHeightMin == 0 || zFaceHeightMin > _proyectable_cloud->points[index].z)
				 zFaceHeightMin = _proyectable_cloud->points[index].z;
			 if (zFaceHeightMax == 0 || zFaceHeightMax < _proyectable_cloud->points[index].z)
				 zFaceHeightMax = _proyectable_cloud->points[index].z;
		 }
	 }

	location.measure_xmin = xFaceHeightMin;
	location.measure_xmax = xFaceHeightMax;
	location.measure_ymin = yFaceHeightMin;
	location.measure_ymax = yFaceHeightMax;

return location;
}

MeasurePoint KeyPointsEstimator::GetHeadMeasureLocation(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> _proyectable_cloud,CvPoint neck_a,CvPoint neck_b,CvPoint head_point)
{
	MeasurePoint location;

	float yFaceHeight = (MIN(neck_a.y,neck_b.y) -  head_point.y)/10 + head_point.y;
		 
	float  yFaceHeightMin = 0;
	float yFaceHeightMax = 0;
	
	for(int i = 0; i <= IMAGE_WIDTH;i++)
	for(int j =  yFaceHeight-3;j <= yFaceHeight+5; j++)
	 {
		 int index  = j*IMAGE_WIDTH+i;
		 idxs.push_back(index);

		 if(index < proyectable_cloud->size())
		 if (proyectable_cloud->points[index].y != 0){
		 if ( yFaceHeightMin == 0 || yFaceHeightMin > proyectable_cloud->points[index].y)
			 yFaceHeightMin = proyectable_cloud->points[index].y;
		 if (yFaceHeightMax == 0 || yFaceHeightMax < proyectable_cloud->points[index].y)
			 yFaceHeightMax = proyectable_cloud->points[index].y;
		 }
	 }

		 for(int i = 0; i < idxs.size();i++)
		 {
			int idx = cloudIdxRel[idxs[i]];
			model_cloud->points[idx].r = proyectable_cloud->points[idxs[i]].r;
			model_cloud->points[idx].g = proyectable_cloud->points[idxs[i]].g;
			model_cloud->points[idx].b = proyectable_cloud->points[idxs[i]].b;
		}
		 
		 
		 location.measure_ymin = yFaceHeightMin;
		 location.measure_ymax = yFaceHeightMax;
		 location.measure_xmin = -1000;
		 location.measure_xmax = 1000;

	return location;
}

MeasurePoint KeyPointsEstimator::GetShouldersLocation(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> _proyectable_cloud,CvPoint a, CvPoint b)
{
	MeasurePoint location;
	float Result = 0;
	float Result2 = 100000;
	
	for(int i = MIN(a.x,b.x); i <= MAX(a.x,b.x);i++)
		 for(int j =  MIN(a.y,b.y);j <= MAX(a.y,b.y); j++)

	 {
		 int index  = j*IMAGE_WIDTH+i;
		 _proyectable_cloud->points[index].b = 255;
		 idxs.push_back(index);
	 
	 }
	 
		 for(int i = 0; i < _proyectable_cloud->size();i++)
		 {
		 if(_proyectable_cloud->points[i].b == 255)
		 {
		 Result = _proyectable_cloud->points[i].x;
		   if(Result < Result2)
		   {
		   Result2 = Result;
		   location.measure_xmin = Result;
		   }
		 
		 }
		 
		 }

		 Result = 0, Result2 = 0;
		 for(int i = 0; i < _proyectable_cloud->size();i++)
		 {
		 if(_proyectable_cloud->points[i].b == 255)
		 {
		 Result = _proyectable_cloud->points[i].x;
		 location.measure_ymax = location.measure_ymin = _proyectable_cloud->points[i].y;
		 
		   if(Result > Result2)
		   {
		   Result2 = Result;
		   location.measure_xmax = Result;
		   }
		 
		 }
		 
		 }

		 

	return location;
}

MeasurePoint * KeyPointsEstimator::GetKeyPoints()
{
  
  MeasurePoint * KeyPoints = new MeasurePoint[6];

  makeProjectable();  // Haciendo la nube de puntos del modelo 3d del usuario, proyectable.
  
    make_side_Projectable(); // Haciendo la nube de puntos del modelo 3d del usuario, proyectable. (para vista lateral)

  KeyPoints2DEstimator * My2DKeyPointsEstimator = new KeyPoints2DEstimator(proyectable_cloud,side_proyectable_cloud); // Creando un nuevo estimador de puntos claves sobre la imagen que representa el modelo 3D

  CvPoint * KeyPoints2D = new CvPoint[10]; // Reservando memoria para los puntos claves bidimensionales

  KeyPoints2D = My2DKeyPointsEstimator->Get2DKeyPoints(); // Hallando los puntos claves bidimensionales.

  if(KeyPoints2D == NULL)
  {
	  KeyPoints = NULL;
	  return KeyPoints;
  }
  
  double angle = atan2((double)-(KeyPoints2D[6].y-KeyPoints2D[2].y),(double)(KeyPoints2D[2].x-KeyPoints2D[6].x))*180/M_PI;

  cout << endl << "Angulo del cuello " << angle << endl;

  ////////////////////// Obtener los puntos donde realizar las mediciones sobre el modelo 3D del usuario.///////////////////////////////////////////////
  KeyPoints[5] = GetMeasureLocation(proyectable_cloud,KeyPoints2D[9],KeyPoints2D[9]); // Punto clave de la entrepierna.
  KeyPoints[3] = GetMeasureLocation(proyectable_cloud,KeyPoints2D[0],KeyPoints2D[1]); // Puntos claves de las axilas.
  KeyPoints[1] = GetMeasureLocation(proyectable_cloud,KeyPoints2D[6],KeyPoints2D[7]);  // Puntos claves del cuello.
  KeyPoints[4] = GetMeasureLocation(proyectable_cloud,KeyPoints2D[4],KeyPoints2D[5]);  // Puntos claves de la cintura.
  KeyPoints[0] = GetHeadMeasureLocation(proyectable_cloud,KeyPoints2D[6],KeyPoints2D[7],KeyPoints2D[8]); // Puntos claves de la cabeza.
  KeyPoints[2] = GetShouldersLocation(proyectable_cloud, KeyPoints2D[2],KeyPoints2D[3]); // Puntos claves de los hombros.
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  cout << "Puntos claves tridimensionales obtenidos de forma correcta..." << endl;

  return KeyPoints;
}

