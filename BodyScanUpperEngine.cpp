#include "BodyScanUpperEngine.h"
//#include <cuda_runtime.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ros/conversions.h>


extern "C"  float * Test(float * X, float * Y, float * Z, int N);
extern "C"  bool * AreValid(float p_x,float p_y, float p_Z,float * X,float * Y, float * Z,int N);

void 
viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 1.05, 1.05);

    pcl::PointXYZ origen;
 /*   origen.x = x1;
    origen.y = y1;
    origen.z = 0;
	
	pcl::PointXYZ dest;
	dest.x = x2;
	dest.y = y2;
	dest.z = 0;

	viewer.addLine(origen,dest,"line");*/
   // std::cout << "i only run once" << std::endl;
    
}


BodyScanUpperEngine::BodyScanUpperEngine(string model_path, int model_orientation, std::vector<char> transforms):user_model_cloud(new (pcl::PointCloud<pcl::PointXYZRGBA>))
{
	float  * X;
	float * Y;
	float * Z;

	

//	pcl::io::loadPLYFile(model_path,*user_model_cloud);
//	pcl::io::loadPCDFile(model_path,*user_model_cloud);
//pcl::io::loadPolygonFile("path_to_file.obj",mesh); 	

	pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFile(model_path,mesh); 

	pcl::fromROSMsg(mesh.cloud,*user_model_cloud);

	
	// Compute principal directions
Eigen::Vector4f pcaCentroid;
pcl::compute3DCentroid(*user_model_cloud, pcaCentroid);
Eigen::Matrix3f covariance;
computeCovarianceMatrixNormalized(*user_model_cloud, pcaCentroid, covariance);
Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
                                                                                ///    the signs are different and the box doesn't get correctly oriented in some cases.
/*// Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PCA<pcl::PointXYZ> pca;
pca.setInputCloud(cloudSegmented);
pca.project(*cloudSegmented, *cloudPCAprojection);
std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;
std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;
// In this case, pca.getEigenVectors() gives similar eigenVectors to eigenVectorsPCA.
*/

// Transform the original cloud to the origin where the principal components correspond to the axes.
Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZRGBA>);

pcl::transformPointCloud(*user_model_cloud, *user_model_cloud, projectionTransform);

//pcl::transformPointCloud(*user_model_cloud, *cloudPointsProjected, projectionTransform);

// Get the minimum and maximum points of the transformed cloud.
pcl::PointXYZRGBA minPoint, maxPoint;
pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

// Final transform
const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

Eigen::Affine3f transform = Eigen::Affine3f::Identity();
transform.rotate (bboxQuaternion);

//Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
//transform2.translation() << 0.0, 1.0, 1.7;

//pcl::transformPointCloud (*user_model_cloud, *user_model_cloud, transform2);

		float theta = -1.57f;

		Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
		transform2.rotate (Eigen::AngleAxisf (-theta, Eigen::Vector3f::UnitZ()));

		pcl::transformPointCloud (*user_model_cloud, *user_model_cloud, transform2);

		Eigen::Affine3f transform3 = Eigen::Affine3f::Identity();
		transform3.rotate (Eigen::AngleAxisf (-theta, Eigen::Vector3f::UnitX()));

		pcl::transformPointCloud (*user_model_cloud, *user_model_cloud, transform3);

				

		pcl::getMinMax3D(*user_model_cloud, minPoint, maxPoint);

		Eigen::Affine3f transform4 = Eigen::Affine3f::Identity();
		
		transform4.translation() << 0.0, 1-maxPoint.y, 1.7;

        pcl::transformPointCloud (*user_model_cloud, *user_model_cloud, transform4);

		pcl::compute3DCentroid(*user_model_cloud, pcaCentroid);


		KeyPointsEstimator * TestEstimator = new KeyPointsEstimator(user_model_cloud);
		
		if(TestEstimator->GetKeyPoints() == NULL)
			{
		
				float theta = 3.1415f;

				Eigen::Affine3f transform5 = Eigen::Affine3f::Identity();
				transform5.rotate (Eigen::AngleAxisf (-theta, Eigen::Vector3f::UnitZ()));

				pcl::transformPointCloud (*user_model_cloud, *user_model_cloud, transform5);				


				pcl::getMinMax3D(*user_model_cloud, minPoint, maxPoint);

		Eigen::Affine3f transform6 = Eigen::Affine3f::Identity();
		
		transform6.translation() << 0.0, 1-maxPoint.y,0;

        pcl::transformPointCloud (*user_model_cloud, *user_model_cloud, transform6);

		
			}


	if(model_orientation == 0)
	{
		float theta = 1.57f;

		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

		pcl::transformPointCloud (*user_model_cloud, *user_model_cloud, transform);
				
		theta  = 180.0f*M_PI/180.0f;

		Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
		transform2.translation() << 0.0, 0.0, 1.30;
	//	transform2.rotate (bboxQuaternion);

		pcl::transformPointCloud (*user_model_cloud, *user_model_cloud, transform2);
	}
	else if(model_orientation == 1)
	{
	
	/*	float theta = 180.0f*M_PI/180.0f;

		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

		pcl::transformPointCloud (*user_model_cloud, *user_model_cloud, transform);
						
		Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
		transform2.translation() << 0.0, 0.0, -1.30;
		
		pcl::transformPointCloud (*user_model_cloud, *user_model_cloud, transform2);*/
	
	}
	else if(model_orientation == -1)
	{
	
		cout << transforms.size();

		

		for(int i = 0; i < transforms.size();i++)
		{
			Eigen::Affine3f current_tranform = Eigen::Affine3f::Identity();

			cout << transforms[i] << endl;

			if(transforms[i] == 'w')
				current_tranform.translation() << 0.0, 0.0, 0.1;
			else if(transforms[i] == 's')
				current_tranform.translation() << 0.0, 0.0, -0.1;
			else if(transforms[i] == 'i')
				current_tranform.rotate (Eigen::AngleAxisf (5*M_PI/180, Eigen::Vector3f::UnitZ()));
			else if(transforms[i] == 'k')
				current_tranform.rotate (Eigen::AngleAxisf (-5*M_PI/180, Eigen::Vector3f::UnitZ()));
			else if(transforms[i] == 'l')
				current_tranform.rotate (Eigen::AngleAxisf (5*M_PI/180, Eigen::Vector3f::UnitY()));
			else if(transforms[i] == 'j')
				current_tranform.rotate (Eigen::AngleAxisf (-5*M_PI/180, Eigen::Vector3f::UnitY()));
		
			pcl::transformPointCloud (*user_model_cloud, *user_model_cloud,current_tranform);
		}
		
		
	
	}
	
	if(IS_DEBUGGING)
	{
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	
	//This will only get called once
    viewer.runOnVisualizationThreadOnce (viewerOneOff);


    
    //blocks until the cloud is actually rendered
    viewer.showCloud(user_model_cloud);

	getchar();
	}
    
	MyPointsEstimator = new KeyPointsEstimator(user_model_cloud);

	MyMeasureMaker = new MeasuresMaker(user_model_cloud);

	measures = new Measure[14];

}


BodyScanUpperEngine::BodyScanUpperEngine(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> transformed_cloud):user_model_cloud(new (pcl::PointCloud<pcl::PointXYZRGBA>))
{
	float  * X;
	float * Y;
	float * Z;

	pcl::copyPointCloud(*transformed_cloud,*user_model_cloud);
	
	for(int i = 0; i < user_model_cloud->size();i++)
			{
				user_model_cloud->points[i].r = 250;
				user_model_cloud->points[i].g = 250;
				user_model_cloud->points[i].b = 250;
			}

	if(IS_DEBUGGING)
	{
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	
	//This will only get called once
    viewer.runOnVisualizationThreadOnce (viewerOneOff);


    
    //blocks until the cloud is actually rendered
    viewer.showCloud(user_model_cloud);

	getchar();
	}
    
	MyPointsEstimator = new KeyPointsEstimator(user_model_cloud);

	

	MyMeasureMaker = new MeasuresMaker(user_model_cloud);

	measures = new Measure[14];

}


BodyScanUpperEngine::~BodyScanUpperEngine(void)
{
	delete measures;
}

Measure BodyScanUpperEngine::findHeight(pcl::PointCloud<pcl::PointXYZRGBA> &cloud_host_, float _sensorHeight)
{
Measure height;

pcl::PointXYZRGBA* hp = Utils::findLowestPoint(Utils::cloudToArray(cloud_host_),cloud_host_.points.size(),Utils::CoordAxis::Y_AXIS);
pcl::PointXYZRGBA* lp = Utils::findHighestPoint(Utils::cloudToArray(cloud_host_),cloud_host_.points.size(),Utils::CoordAxis::Y_AXIS);
	
		height.x = hp->x;
		height.y = hp->y;
		height.z = hp->z;
		height.radius =  0;	

    	height.Value = abs(hp->y)+abs(lp->y);
    


return height;
}

float BodyScanUpperEngine::GetCalfHeight(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> cloud, float crotch_height)
{
	float calf_height = 0;
	float min_altura = Utils::findHighestPoint(Utils::cloudToArray(*cloud),cloud->size(),Utils::CoordAxis::Y_AXIS)->y;
	float temp_z = -1000;
	
	for(int i = 0; i < user_model_cloud->size();i++)
	{
		if(cloud->points[i].y > crotch_height && cloud->points[i].y < min_altura-0.05)
		{
			if(cloud->points[i].z > temp_z)
			{
				temp_z = cloud->points[i].z;
				calf_height = cloud->points[i].y;
			}
		
		}
	
	}



return calf_height;
}

float BodyScanUpperEngine::GetButtocksHeight(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> cloud,float crotch_height, float waist_height)
{
  float buttocks_height = 0;

  float min_altura = Utils::findHighestPoint(Utils::cloudToArray(*cloud),cloud->size(),Utils::CoordAxis::Y_AXIS)->y;
	float temp_z = -1000;
	
	for(int i = 0; i < user_model_cloud->size();i++)
	{
		if(cloud->points[i].y < crotch_height && cloud->points[i].y > waist_height)
		{
			if(cloud->points[i].z > temp_z)
			{
				temp_z = cloud->points[i].z;
				buttocks_height = cloud->points[i].y;
			}
		
		}
	
	}


  return buttocks_height;
}


void BodyScanUpperEngine::UnpaintCloud()
{
	for(int i = 0; i < user_model_cloud->size();i++)
	{
		user_model_cloud->points[i].r = 250;
	    user_model_cloud->points[i].g = 250;
		user_model_cloud->points[i].b = 250;
		
	}
  


}



bool BodyScanUpperEngine::ProcessModel()
{
	
	
	measures[0] = findHeight(*user_model_cloud,sensor_height); // Calulando la altura del modelo.

	MeasurePoint * KeyPoint3D = new MeasurePoint[5]; // Reservando memoria para los puntos claves.

	KeyPoint3D =  MyPointsEstimator->GetKeyPoints(); // Calculando los puntos claves necesarios para obtener las mediciones sobre el modelo 3d.

	if(KeyPoint3D == NULL)
		return false;

	cout << "Calculando medidas..." << endl;

	Plane head_plane; //Plano resultante de la medición de la cabeza.
	measures[1] = MyMeasureMaker->GetMeasure(user_model_cloud,head_plane,KeyPoint3D[1].measure_xmin+(KeyPoint3D[1].measure_xmax-KeyPoint3D[1].measure_xmin)/2, 
				KeyPoint3D[0].measure_ymax+0.01*measures[0].Value,0,-1000,1000,'n',0,0.003,0.05); // Obteniendo la medida de la cabeza

	// Obteniendo valor y ubicación de la medida de los hombros.
	measures[3].Value = abs(KeyPoint3D[2].measure_xmax) + abs(KeyPoint3D[2].measure_xmin);  // Calculando el ancho de los hombros
	measures[3].y = KeyPoint3D[2].measure_ymax;
	measures[3].x = (KeyPoint3D[2].measure_xmax+KeyPoint3D[2].measure_xmin)/2;
	measures[3].radius = (KeyPoint3D[2].measure_xmax-KeyPoint3D[2].measure_xmin)/2;
	
	// Obteniendo valor y ubicación de la medición de la cintura
	float waist_height = 0;
	float chest_height = KeyPoint3D[3].measure_ymin - (abs(KeyPoint3D[3].measure_ymax) - abs(KeyPoint3D[3].measure_ymin))/2; // Obteniendo la altura de la medición del pecho
																															 // a partir de los puntos claves del pecho.
	Plane waist_plane;				
	waist_height = KeyPoint3D[0].measure_ymax+(measures[0].Value*0.34); // Hallando lugar de la cintura mediante relación antropométrica
	
	measures[5] = MyMeasureMaker->GetMeasure(user_model_cloud,waist_plane,KeyPoint3D[4].measure_xmin+(KeyPoint3D[4].measure_xmax-KeyPoint3D[4].measure_xmin)/2, 
											waist_height,0,-1000,1000,'n',0,0.003,0.05); // Obteniendo valor de la medición a partir de relación antropométrica
	
	
	measures[6] = MyMeasureMaker->GetMeasure(user_model_cloud,waist_plane,KeyPoint3D[5].measure_xmin+(KeyPoint3D[5].measure_xmax-KeyPoint3D[5].measure_xmin)/2, 
											 KeyPoint3D[5].measure_ymax,0, KeyPoint3D[5].measure_xmin,1000,'n',0,0.003,0.05); // Obteniendo valor de la medición de la posición de la entrepierna

	measures[7] = MyMeasureMaker->GetMeasure(user_model_cloud,waist_plane,KeyPoint3D[5].measure_xmin+(KeyPoint3D[5].measure_xmax-KeyPoint3D[5].measure_xmin)/2,
				  waist_height + measures[0].Value*0.08,0,-1000,1000,'n',0,0.003,0.05); // Obteniendo valor de la medición de la cadera a partir de relaci'on antropométrica con la cintura y la altura.
	

	//Calculando altura para la medición del cuello
	float neck_height = KeyPoint3D[1].measure_ymin - (abs(KeyPoint3D[1].measure_ymax) - abs(KeyPoint3D[1].measure_ymin))/3;
	
	
	// Iniciando hilo para la medición del cuello
	NeckMeasureThread * MyNeckThread = new NeckMeasureThread(user_model_cloud,KeyPoint3D[4].measure_xmin,KeyPoint3D[4].measure_xmax,neck_height);
	HANDLE   hth1;
    unsigned  uiThread1ID;
	hth1 = (HANDLE)_beginthreadex( NULL,         // security
                                   0,            // stack size
								   NeckMeasureThread::ThreadStaticEntryPoint,
                                   MyNeckThread,           // arg list
                                   CREATE_SUSPENDED,  // so we can later call ResumeThread()
                                   &uiThread1ID );
	ResumeThread( hth1 ); // inicia

	
	// Iniciando el hilo para la medición del pecho
	ChestMeasureThread * MyChestThread = new ChestMeasureThread(user_model_cloud,KeyPoint3D[3].measure_xmin,KeyPoint3D[3].measure_xmax,
										 chest_height,KeyPoint3D[2].measure_xmin,KeyPoint3D[2].measure_xmax,'c');
	HANDLE   hth2;
    unsigned  uiThread2ID;
	hth2 = (HANDLE)_beginthreadex( NULL,         // security
                                   0,            // stack size
								   ChestMeasureThread::ChestThreadStaticEntryPoint,
                                   MyChestThread,           // arg list
                                   CREATE_SUSPENDED,  // so we can later call ResumeThread()
                                   &uiThread1ID );
	ResumeThread( hth2 ); // inicia hilo para la medición del pecho

	// Iniciando hilo para la medición del muslo
	CrunchMeasureThread * MyCrunchThread = new CrunchMeasureThread(user_model_cloud,KeyPoint3D[5].measure_xmin,KeyPoint3D[5].measure_xmin+(KeyPoint3D[4].measure_xmax-KeyPoint3D[5].measure_xmin)/2,
																   KeyPoint3D[5].measure_ymax,KeyPoint3D[5].measure_xmin,1000,'c');
	HANDLE   hth3;
    unsigned  uiThread3ID;
	hth3 = (HANDLE)_beginthreadex( NULL,         // security
                                   0,            // stack size
								   CrunchMeasureThread::ThreadStaticEntryPoint,
                                   MyCrunchThread,           // arg list
                                   CREATE_SUSPENDED,  // so we can later call ResumeThread()
                                   &uiThread3ID );

	ResumeThread( hth3 ); // inicia hilo para la medición del muslo.


	///////////////////////////////////////////////////////////////////////////////////////
	WaitForSingleObject( hth1, INFINITE ); 
	WaitForSingleObject( hth2, INFINITE ); 
	WaitForSingleObject( hth3, INFINITE ); 

	measures[2] = MyNeckThread->GetNeck(); // Obtener la medición del  cuello.
	Plane chest_plane;
	measures[4] = MyChestThread->GetChest(); // Obtener la medición del pecho (perímetro).
	neck_plane = MyNeckThread->GetNeckPlane();

	measures[6] = MyCrunchThread->GetCrunch(); // Obtener la medición del muslo.

	float pantorrilla_altura = GetCalfHeight(user_model_cloud,measures[6].y);

	measures[8] = MyMeasureMaker->GetMeasure(user_model_cloud,waist_plane,measures[6].x,pantorrilla_altura,measures[6].z,-1000,1000,'n',0.0,0.002,0.05); // Obtener la medición de la pantorrilla.

	float nalgas_height = GetButtocksHeight(user_model_cloud,measures[6].y,waist_height);

	measures[9] = MyMeasureMaker->GetMeasure(user_model_cloud,waist_plane,KeyPoint3D[4].measure_xmin+(KeyPoint3D[4].measure_xmax-KeyPoint3D[4].measure_xmin)/2,nalgas_height,0,-1000,1000,'n',0,0.003,0.05); // Obtener medición de las nalgas.

//	measures[10] = MyMeasureMaker->GetMeasure(user_model_cloud,waist_plane,measures[8].x,pantorrilla_altura+measures[0].Value*0.15,measures[8].z,-1000,1000,'n',0.0,0.002,0.05); // Obtener la medición del tobillo
	
	//Obteniendo la medición del ancho del pecho
	measures[10] = measures[4];
	measures[10].Value = abs(measures[4].xMin) + abs(measures[4].xMax);
	measures[10].y = measures[4].y;
	
	//Obteniendo el ángulo cuello-hombro KeyPoint3D[2]: hombro ------- KeyPoint3D[4]: cuello
	measures[11].Value = atan2(KeyPoint3D[2].measure_ymax-KeyPoint3D[4].measure_ymax,KeyPoint3D[2].measure_xmax-KeyPoint3D[4].measure_xmax)*180/M_PI;


	cout << "Todas las medidas han sido calculadas de forma satisfactoria..." << endl;


	///////////////////////////////////////////Región para pintar, solo con propósitos de debuggeo

	for(int i = 0; i < user_model_cloud->size();i++)
			{
				user_model_cloud->points[i].r = 250;
				user_model_cloud->points[i].g = 250;
				user_model_cloud->points[i].b = 250;
			}

	if(1)
		{

			for(int i = 0; i < user_model_cloud->size();i++)
			{
				user_model_cloud->points[i].r = 200;
				user_model_cloud->points[i].g = 200;
				user_model_cloud->points[i].b = 200;
			}
			
		for(int i = 0; i < user_model_cloud->size();i++)
			if(user_model_cloud->points[i].x >= measures[1].xMax && user_model_cloud->points[i].x <= measures[1].xMin
				&& user_model_cloud->points[i].y >= measures[1].y - 0.01 && user_model_cloud->points[i].y <= measures[1].y + 0.01)
				user_model_cloud->points[i].b = 255;

		for(int i = 0; i < user_model_cloud->size();i++)
			if(user_model_cloud->points[i].x >= measures[2].xMax && user_model_cloud->points[i].x <= measures[2].xMin
				&& user_model_cloud->points[i].y >= measures[2].y - 0.01 && user_model_cloud->points[i].y <= measures[2].y + 0.01)
			{
				user_model_cloud->points[i].b = 255;
				user_model_cloud->points[i].r = 255;
			}

		for(int i = 0; i < user_model_cloud->size();i++)
			if(user_model_cloud->points[i].x >= measures[5].xMax && user_model_cloud->points[i].x <= measures[5].xMin
				&& user_model_cloud->points[i].y >= measures[5].y - 0.01 && user_model_cloud->points[i].y <= measures[5].y + 0.01)
				user_model_cloud->points[i].b = 255;
		

		for(int i = 0; i < user_model_cloud->size();i++)
			if(user_model_cloud->points[i].x >= measures[4].xMax && user_model_cloud->points[i].x <= measures[4].xMin
				&& user_model_cloud->points[i].y >= measures[4].y - 0.01 && user_model_cloud->points[i].y <= measures[4].y + 0.01)
				user_model_cloud->points[i].r = 255;

		for(int i = 0; i < user_model_cloud->size();i++)
			if(user_model_cloud->points[i].x >= measures[6].xMax && user_model_cloud->points[i].x <= measures[6].xMin
				&& user_model_cloud->points[i].y >= measures[6].y - 0.01 && user_model_cloud->points[i].y <= measures[6].y + 0.01)
			{
				user_model_cloud->points[i].r = 50;
				user_model_cloud->points[i].g = 255;
				user_model_cloud->points[i].b = 255;
			}

		for(int i = 0; i < user_model_cloud->size();i++)
			if(user_model_cloud->points[i].x >= measures[7].xMax && user_model_cloud->points[i].x <= measures[7].xMin
				&& user_model_cloud->points[i].y >= measures[7].y - 0.01 && user_model_cloud->points[i].y <= measures[7].y + 0.01)
			{
				user_model_cloud->points[i].r = 255;
				user_model_cloud->points[i].g = 100;
				user_model_cloud->points[i].b = 100;
			}

			for(int i = 0; i < user_model_cloud->size();i++)
			if(user_model_cloud->points[i].x >= measures[8].xMax && user_model_cloud->points[i].x <= measures[8].xMin
				&& user_model_cloud->points[i].y >= measures[8].y - 0.01 && user_model_cloud->points[i].y <= measures[8].y + 0.01)
			{
				user_model_cloud->points[i].r = 0;
				user_model_cloud->points[i].g = 255;
				user_model_cloud->points[i].b = 0;
			}

			for(int i = 0; i < user_model_cloud->size();i++)
			if(user_model_cloud->points[i].x >= measures[9].xMax && user_model_cloud->points[i].x <= measures[9].xMin
				&& user_model_cloud->points[i].y >= measures[9].y - 0.01 && user_model_cloud->points[i].y <= measures[9].y + 0.01)
			{
				user_model_cloud->points[i].r = 100;
				user_model_cloud->points[i].g = 255;
				user_model_cloud->points[i].b = 22;
			}

			for(int i = 0; i < user_model_cloud->size();i++)
			if(user_model_cloud->points[i].x >= measures[10].xMax && user_model_cloud->points[i].x <= measures[10].xMin
				&& user_model_cloud->points[i].y >= measures[10].y - 0.01 && user_model_cloud->points[i].y <= measures[10].y + 0.01)
			{
				user_model_cloud->points[i].r = 100;
				user_model_cloud->points[i].g = 255;
				user_model_cloud->points[i].b = 22;
			}
		
	/*	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	    
		//This will only get called once
		viewer.runOnVisualizationThreadOnce (viewerOneOff);

		//blocks until the cloud is actually rendered
		viewer.showCloud(user_model_cloud);
	
		getchar();*/
	}
	
	return true;
}

