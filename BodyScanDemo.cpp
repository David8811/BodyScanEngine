
#include "BodyScanUpperEngine.h"
#include <iostream>
//#include <cuda_runtime.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>


#include <conio.h>
#include <stdio.h>

bool update = false;
bool plane_update;
bool do_automatic_measurements = false;
bool switch_mesh_plane_movement = true;

std::vector<Measure> drawing_measures;
std::vector<Plane> drawing_planes;
std::vector<char> transforms;
BodyScanUpperEngine * MyEngine;

pcl::ModelCoefficients::Ptr plane_1 (new pcl::ModelCoefficients);

float x_translation, y_translation,z_translation = 0;
float measurament_plane_size = 0.35;
float measurament_plane_thickness = 0.005;

float Pitch = 0;
float Yaw = 0;
float Roll = 0;

std::vector<Eigen::Affine3f> my_transforms;

Eigen::Affine3f current_tranform = Eigen::Affine3f::Identity();


void mouseIIEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* cloud_void)
{
 
	if(event.MouseMove)
	{

		x_translation  = event.getX();
		y_translation  = event.getY();
	    
		plane_update = true;
	
	}

}

void mouseEventOccurred (const pcl::visualization::PointPickingEvent &event,
                         void* cloud_void)
{
	  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> cloud = *static_cast<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> *> (cloud_void);  

	  event.getPoint(x_translation,y_translation,z_translation);	

	  current_tranform = Eigen::Affine3f::Identity();

	  current_tranform.translation() << x_translation, y_translation, z_translation;

	  cout << x_translation << endl;

	  plane_update = true;
}




void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* cloud_void)
{
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> cloud = *static_cast<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> *> (cloud_void);

  float angle_desplacement, translation_displacement;

  if(event.isCtrlPressed())
  {
    
    angle_desplacement = 15*M_PI/180;
	translation_displacement = 0.1;
  }
  else
  {
  
	angle_desplacement = 2*M_PI/180;
	translation_displacement = 0.01;
  
  }

  
  if (event.getKeySym () == "r" && event.keyDown ())
  {
	  drawing_measures.clear();
	  drawing_planes.clear();
	  cout << "reseting all measures" << endl;

	  for(int i = 0; i < cloud->size();i++)
		  {
												
	    	cloud->points[i].r = 250;
		    cloud->points[i].g = 250;
			cloud->points[i].b = 250;				
		  }							 
	  update = true;
  }

   if(event.getKeySym () == "x" && event.keyDown ())
  {
	  switch_mesh_plane_movement = !switch_mesh_plane_movement;
	  plane_update = true;

   }

    if(event.getKeySym () == "t" && event.keyDown ())
  {
	  current_tranform = Eigen::Affine3f::Identity();
	  current_tranform.translation() << 0, -translation_displacement,0;

	   y_translation -= translation_displacement;

	  plane_update = true;

   }

	if(event.getKeySym () == "g" && event.keyDown ())
  {
	  current_tranform = Eigen::Affine3f::Identity();
	  current_tranform.translation() << 0, translation_displacement,0;

	  y_translation += translation_displacement;

	  plane_update = true;

   }


  if(event.getKeySym () == "u" && event.keyDown ())
  {
	  if(drawing_measures.size() > 0)
	  {
		  for(int i = 0; i < cloud->size();i++)
		  {
				if(cloud->points[i].x >= drawing_measures[drawing_measures.size()-1].xMax && cloud->points[i].x <=drawing_measures[drawing_measures.size()-1].xMin
					&& drawing_planes[i].Contains(cloud->points[i],0.005))
				{								
						cloud->points[i].r = 250;
						cloud->points[i].g = 250;
						cloud->points[i].b = 250;
				}
		  }
		  drawing_measures.erase(drawing_measures.begin()+drawing_measures.size()-1);	  
		  drawing_planes.erase(drawing_planes.begin()+drawing_planes.size()-1);	  
		  cout << "erasing last measure" << endl;
		  update = true;
	  }
  }


  if(event.getKeySym () == "m" && event.keyDown ())
	  {
		 MeasuresMaker * MyMeasureMaker = new MeasuresMaker(cloud);

		 float angle = 0;

		 
		 angle = -(2*Yaw*180/M_PI);		 


		Plane measure_plane; //Plano resultante de la medición de la cabeza.
		Measure measure =  MyMeasureMaker->GetMeasure(cloud,measure_plane,x_translation, y_translation,z_translation,x_translation-measurament_plane_size/2,x_translation+measurament_plane_size/2,'m',angle,0.005,0.02); // Obteniendo la medida de la cabeza

		drawing_measures.push_back(measure);
		drawing_planes.push_back(measure_plane);

		cout << "*MU " << measure.Value << endl;
			
		update = true;
	  }

	if(event.getKeySym () == "d" && event.keyDown ())
	  {
		 x_translation += translation_displacement;

		 current_tranform = Eigen::Affine3f::Identity();
		  
		  current_tranform.translation() << translation_displacement, 0.0, 0.0;

		 plane_update = true;
	  }

	if(event.getKeySym () == "a" && event.keyDown ())
	  {
		   x_translation -= translation_displacement;

		   current_tranform = Eigen::Affine3f::Identity();
		  
		  current_tranform.translation() << -translation_displacement, 0.0, 0.0;

		 plane_update = true;
	  }


	  if(event.getKeySym () == "w" && event.keyDown ())
	  {
		  current_tranform = Eigen::Affine3f::Identity();
		  
		  current_tranform.translation() << 0.0, 0.0, translation_displacement;

		  transforms.push_back('w');

		  z_translation += translation_displacement;

		 plane_update = true;
	  }


	  if(event.getKeySym () == "s" && event.keyDown ())
	  {
		  current_tranform = Eigen::Affine3f::Identity();
		  
		  current_tranform.translation() << 0.0, 0.0, -translation_displacement;

		  transforms.push_back('s');

		  z_translation -= translation_displacement;

		 plane_update = true;
	  }

	 if(event.getKeySym () == "p" && event.keyDown ())
	  {
		  measurament_plane_size += 0.025;
		 plane_update = true;
	  }

	 if(event.getKeySym () == "o" && event.keyDown ())
	  {
		   measurament_plane_size -= 0.025;
		 plane_update = true;
	  }


	 if(event.getKeySym () == "z" && event.keyDown ())
	  {
		 
		  Eigen::Affine3f transform1 = Eigen::Affine3f::Identity();
		 		  
		  transform1.translation() << 0,0,-3.4;

		  my_transforms.push_back(transform1);


		  current_tranform = Eigen::Affine3f::Identity();
		  
		  current_tranform.rotate (Eigen::AngleAxisf (-3.1415, Eigen::Vector3f::UnitY()));
		  
		  my_transforms.push_back(current_tranform);

		 
		  
		 plane_update = true;
	  }

	 if(event.getKeySym () == "c" && event.keyDown ())
	  {
		 
		   

		  current_tranform = Eigen::Affine3f::Identity();
		  
		  current_tranform.rotate (Eigen::AngleAxisf (-3.1415, Eigen::Vector3f::UnitZ()));
		  
		  my_transforms.push_back(current_tranform);

	    

		 		  
		 plane_update = true;
	  }



	 if(event.getKeySym () == "i" && event.keyDown ())
	  {
		  Yaw += angle_desplacement;

		  current_tranform = Eigen::Affine3f::Identity();
		  
		  current_tranform.rotate (Eigen::AngleAxisf (5*M_PI/180, Eigen::Vector3f::UnitZ()));

		 
		  if(Yaw <= 360)
			  Yaw += angle_desplacement;
		  else
			  Yaw = 0;


		 plane_update = true;
	  }

	 if(event.getKeySym () == "k" && event.keyDown ())
	  {
		  
		  Yaw -= angle_desplacement;

		  current_tranform = Eigen::Affine3f::Identity();
		  
		  current_tranform.rotate (Eigen::AngleAxisf (-angle_desplacement, Eigen::Vector3f::UnitZ()));
		  
		  transforms.push_back('k');

		  if(Yaw >= -360)
		  Yaw -= angle_desplacement;
		  else
			  Yaw = 0;

		 plane_update = true;
	  }

	 if(event.getKeySym () == "j" && event.keyDown ())
	  {
		  
		  current_tranform = Eigen::Affine3f::Identity();
		 
		  current_tranform.rotate (Eigen::AngleAxisf (-angle_desplacement, Eigen::Vector3f::UnitY()));
		  
		  transforms.push_back('j');
		  
		 plane_update = true;
	  }

	 if(event.getKeySym () == "l" && event.keyDown ())
	  {
		  
		  current_tranform = Eigen::Affine3f::Identity();
		  
		  current_tranform.rotate (Eigen::AngleAxisf (angle_desplacement, Eigen::Vector3f::UnitY()));
		  
		  transforms.push_back('l');
		  
		 plane_update = true;
	  }


	 

	 if(event.getKeySym () == "n" && event.keyDown ())
	 {
		 do_automatic_measurements = true;
		 update = true;

	 }



}

boost::shared_ptr<pcl::visualization::PCLVisualizer> customMesh (string mesh_path, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> user_model_cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("BodyScan 3D C++ Demo"));

  viewer->setBackgroundColor (0, 0, 0);

  viewer->addText("Help:\nPut the model where the arrow indicates.\nUse X for switching between plane/model movement.\nW S A D T G for translation.\nI K J L for rotation\nCTRL + translation or rotation keys for fast movement\nAlso use SHIFT+lClick  for translation.\nZ for flipping the model around the Y axis\nC for flipping the model around the Z axis\nN for automatic measurement, M for manual measurements\nP and O increases and decreases plane´s size\nR to reset all measures",10,380,12,255.0,100.0,200.0,"text");

  viewer->setWindowBorders(true);

  pcl::PolygonMesh mesh;
    
  pcl::io::loadPolygonFileSTL(mesh_path,mesh);
  				
  pcl::toROSMsg(*user_model_cloud, mesh.cloud); 

 /* Eigen::Vector3f offset(pose.x(), pose.y(), 0.0);*/
   Eigen::Quaternionf rotation(sin(Roll) * cos(Pitch) * cos(Yaw) - cos(Roll)* 
sin(Pitch) * sin(Yaw),
                       cos(Roll) * sin(Pitch) * cos(Yaw) + sin(Roll) * cos(Pitch)
* sin(Yaw),
                       cos(Roll) * cos(Pitch) * sin(Yaw) - sin(Roll) * sin(Pitch)
* cos(Yaw),
                       cos(Roll) * cos(Pitch) * cos(Yaw) + sin(Roll) * sin(Pitch)
* sin(Yaw)); 

  
  
  Eigen::Vector3f platform_trans_transform(0,1.0,1.7);
  Eigen::Vector3f trans_transform(0,0,0);
  
  Eigen::Quaternion<float> rot_transform = Eigen::Quaternion<float>::Identity();
 
  viewer->addCube(trans_transform,rotation,measurament_plane_size,measurament_plane_thickness,measurament_plane_size,"cube",0);

  viewer->addCube(platform_trans_transform,rotation,measurament_plane_size,0.05,measurament_plane_size,"platform",0);
  
  viewer->addPolygonMesh(mesh,"meshes",0);
  
  viewer->setCameraPosition(0.0,0,-2.0,0,-1,0);

  
  pcl::PointXYZRGB a, b;

  a.x = 0;
  a.y = 1.0;
  a.z =1.7;

  b.x = 0;
  b.y = 0.0;
  b.z = 1.7;

  viewer->addArrow<pcl::PointXYZRGB,pcl::PointXYZRGB>(a,b,255,10,0,false);
  
 // viewer->addCoordinateSystem();
    
 // viewer->addCoordinateSystem (1.0);
  viewer->setRepresentationToSurfaceForAllActors();
  viewer->setShapeRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR,200,0,50,"cube");
  viewer->setShapeRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR,0,100,50,"platform");

 
  
  viewer->initCameraParameters ();

//  viewer->registerMouseCallback(mouseEventOccurred);

  return (viewer);
}


int main (int argc, char * argv[])
{
			
/*	cudaEvent_t start, stop;
	cudaEventCreate(&start);
	cudaEventCreate(&stop);
	cudaEventRecord( start, 0 );*/


	int orientation = atoi(argv[2]);
	std::string _model_path = argv[1];
	
	pcl::PolygonMesh mesh;
		 
    pcl::io::loadPolygonFileSTL(_model_path,mesh);
	
	
	MyEngine = new BodyScanUpperEngine(_model_path,-1,transforms); // Inicializando un nuevo motor.

	

	if(argv[1] != NULL)
	{
		MyEngine->SetSensorHeight(atof(argv[1])); // Configurando la altura en el motor
	}
	else
		MyEngine->SetSensorHeight(1.50);

	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("BodyScan 3D C++ Demo"));
	viewer = customMesh(_model_path,MyEngine->GetUserCloud());

	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> user_model_cloud = MyEngine->GetUserCloud();
	
	viewer->registerPointPickingCallback (mouseEventOccurred, (void*)&user_model_cloud);	
	viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&user_model_cloud);
//	viewer->registerMouseCallback(mouseIIEventOccurred, (void*)&user_model_cloud);


	for(int i = 0; i < user_model_cloud->size();i++)
		  {
												
	    	user_model_cloud->points[i].r = 250;
		    user_model_cloud->points[i].g = 250;
			user_model_cloud->points[i].b = 250;				
		  }							 
	  update = true;

	

	while (!viewer->wasStopped ()){

		viewer->spinOnce (10);
		
		char key = getchar();
		

		if(key == 'z')
		{
		
		  Eigen::Affine3f transform1 = Eigen::Affine3f::Identity();
		 		  
		  transform1.translation() << 0,0,-3.4;

		  my_transforms.push_back(transform1);


		  current_tranform = Eigen::Affine3f::Identity();
		  
		  current_tranform.rotate (Eigen::AngleAxisf (-3.1415, Eigen::Vector3f::UnitY()));
		  
		  my_transforms.push_back(current_tranform);

		 
		  
		 plane_update = true;
		

		
		

		
		}

		if(update)
		{

		viewer->removeShape("cube");

		if(do_automatic_measurements)
		{
		
		Eigen::Vector3f trans_transform(x_translation,y_translation,z_translation);
  
		 Eigen::Quaternionf rotation(sin(Roll) * cos(Pitch) * cos(Yaw) - cos(Roll)* 
			sin(Pitch) * sin(Yaw),
            cos(Roll) * sin(Pitch) * cos(Yaw) + sin(Roll) * cos(Pitch)
			* sin(Yaw),
            cos(Roll) * cos(Pitch) * sin(Yaw) - sin(Roll) * sin(Pitch)
			* cos(Yaw),
            cos(Roll) * cos(Pitch) * cos(Yaw) + sin(Roll) * sin(Pitch)
			* sin(Yaw)); 


		 MyEngine = new BodyScanUpperEngine(user_model_cloud); // Inicializando un nuevo motor.
		MeasuresMaker * MyMeasureMaker = new MeasuresMaker(user_model_cloud); // Arreglar esto por la nube transformada;
		
		
				if(MyEngine->ProcessModel()) // Procesando el modelo 3d del usuario.
			{
					Measure * results = new Measure[11];
					results = MyEngine->GetAllMeasures(); // Pedir todas las medidas al motor.

					

					// Factores para compatibilizar los ejes de coordenadas de la nube de puntos en el motor, con las del componente de visualización en C#
				int xFactor = 1;
				int yFactor = 1;
				int zFactor = -1;
				Plane neckPlane = MyEngine->GetNeckPlane(); // Obteniendo el plano de la medición del cuello, para luego poder representarlo de forma correcta en el visualizador de la aplicación de C#
				// Enviándo mediciones a través del flujo de salida estándar hacia la aplicación de C#	 
				cout << "*M02 "  << results[0].Value << " " << results[0].x  << " " << results[0].y << " " << results[0].z << " " << results[0].radius<< " " << (results[0].xMax>results[0].xMin?results[0].xMax : results[0].xMin )<< " " << (results[0].xMax<results[0].xMin?results[0].xMax : results[0].xMin )<< "" << endl;
				cout << "*M03 "  << results[1].Value   << " " << results[1].x  << " " << results[1].y << " " << results[1].z << " " << results[1].radius << " " << (results[1].xMax>results[1].xMin?results[1].xMax : results[1].xMin ) << " " << (results[1].xMax<results[1].xMin?results[1].xMax : results[1].xMin ) << "" << endl;
				cout << "*M06 "  << results[2].Value   << " " << results[2].x  << " " << results[2].y << " " << results[2].z << " " << results[2].radius << " " << (results[2].xMax>results[2].xMin?results[2].xMax : results[2].xMin ) << " " << (results[2].xMax<results[2].xMin?results[2].xMax : results[2].xMin ) << " " <<  neckPlane.Normal.X*xFactor << " " <<  neckPlane.Normal.Y*yFactor<< " " <<  neckPlane.Normal.Z*zFactor<< " " <<  neckPlane.ReferencePoint.x*xFactor << " " << neckPlane.ReferencePoint.y*yFactor << " " << neckPlane.ReferencePoint.z*zFactor << " "<< 0.005 << "" << endl;
				cout << "*M01 "  << results[3].Value << " " << results[3].x  << " " << results[3].y << " " << results[3].z << " " << results[3].radius <<  " " << (results[3].xMax>results[3].xMin?results[3].xMax : results[3].xMin ) << " " << (results[3].xMax<results[3].xMin?results[3].xMax : results[3].xMin )<< "" << endl;
				cout << "*M04 " << results[4].Value  << " " << results[4].x  << " " << results[4].y << " " << results[4].z << " " << results[4].radius <<  " " << (results[4].xMax>results[4].xMin?results[4].xMax : results[4].xMin ) << " " << (results[4].xMax<results[4].xMin?results[4].xMax : results[4].xMin ) << "" << endl;
				cout << "*M05 " << results[5].Value  << " " << results[5].x  << " " << results[5].y << " " << results[5].z << " " << results[5].radius << " " << (results[5].xMax>results[5].xMin?results[5].xMax : results[5].xMin ) << " " << (results[5].xMax<results[5].xMin?results[5].xMax : results[5].xMin ) <<  "" << endl;
				cout << "*M07 " << results[7].Value  << " " << results[7].x  << " " << results[7].y << " " << results[7].z << " " << results[7].radius << " " << (results[7].xMax>results[7].xMin?results[7].xMax : results[7].xMin ) << " " << (results[7].xMax<results[7].xMin?results[7].xMax : results[7].xMin ) <<  "" << endl;
				cout << "*M08 " << results[6].Value  << " " << results[6].x  << " " << results[6].y << " " << results[6].z << " " << results[6].radius << " " << (results[6].xMax>results[6].xMin?results[6].xMax : results[6].xMin ) << " " << (results[6].xMax<results[6].xMin?results[6].xMax : results[6].xMin ) << "" <<  endl;
				cout << "*M09 " << results[8].Value  << " " << results[8].x  << " " << results[8].y << " " << results[8].z << " " << results[8].radius << " " << (results[8].xMax>results[8].xMin?results[8].xMax : results[8].xMin ) << " " << (results[8].xMax<results[8].xMin?results[8].xMax : results[8].xMin ) << "" <<  endl;
				cout << "*M10 " << results[9].Value  << " " << results[9].x  << " " << results[9].y << " " << results[9].z << " " << results[9].radius << " " << (results[9].xMax>results[9].xMin?results[9].xMax : results[9].xMin ) << " " << (results[9].xMax<results[9].xMin?results[9].xMax : results[9].xMin ) << "" <<  endl;
	

				cout << "Ancho del pecho: " << results[10].Value << endl;
				cout << "Angulo cuello hombro" << results[11].Value << endl;

				drawing_measures.clear();
				drawing_planes.clear();
				
				
				
					user_model_cloud = MyEngine->GetUserCloud();
			}
				else
					cout << "Unable to make automatic measurements" << endl;
		   
				
		}


		viewer->removePolygonMesh("meshes",0); 
		
		if(! do_automatic_measurements)
		for(int i = 0; i < user_model_cloud->size();i++)
		{
			for(int j = 0; j < drawing_measures.size();j++)
			{
				if(user_model_cloud->points[i].x >= drawing_measures[j].xMax && user_model_cloud->points[i].x <=drawing_measures[j].xMin
					&& drawing_planes[j].Contains(user_model_cloud->points[i],0.005))
				{								
						user_model_cloud->points[i].r = 255;
				}
			}

			if(user_model_cloud->points[i].r != 255)
			{
		
				user_model_cloud->points[i].r = 250;
				user_model_cloud->points[i].g = 250;
				user_model_cloud->points[i].b = 250;
		
			}
		}

        pcl::toROSMsg(*user_model_cloud, mesh.cloud); 

         viewer->addPolygonMesh(mesh,"meshes",0);		 
		 update = false;
		 do_automatic_measurements = false;
		}

		if(plane_update)
		{

			if(!switch_mesh_plane_movement)
			{
				Eigen::Vector3f trans_transform(x_translation,y_translation,z_translation);
  
				Eigen::Quaternionf rotation(sin(Roll) * cos(Pitch) * cos(Yaw) - cos(Roll)* 
				sin(Pitch) * sin(Yaw),
				cos(Roll) * sin(Pitch) * cos(Yaw) + sin(Roll) * cos(Pitch)
				* sin(Yaw),
				cos(Roll) * cos(Pitch) * sin(Yaw) - sin(Roll) * sin(Pitch)
				* cos(Yaw),
				cos(Roll) * cos(Pitch) * cos(Yaw) + sin(Roll) * sin(Pitch)
				* sin(Yaw)); 
		

				viewer->removeShape("cube");

				
				viewer->addCube(trans_transform,rotation,measurament_plane_size,measurament_plane_thickness,measurament_plane_size,"cube",0);
				viewer->setShapeRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR,200,0,50,"cube");
				viewer->setRepresentationToSurfaceForAllActors();
			}
			else
			{

   				viewer->removePolygonMesh("meshes",0); 
				 
				for(int i = 0; i < my_transforms.size();i++)
					pcl::transformPointCloud(*user_model_cloud,*user_model_cloud,my_transforms[i]);

				pcl::PointXYZRGBA minPoint, maxPoint;

				pcl::getMinMax3D(*user_model_cloud, minPoint, maxPoint);

				Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		
				transform.translation() << 0.0, 1-maxPoint.y, 0;

				pcl::transformPointCloud(*user_model_cloud,*user_model_cloud,transform);

				my_transforms.clear();

				 pcl::toROSMsg(*user_model_cloud, mesh.cloud); 
				 viewer->addPolygonMesh(mesh,"meshes",0);		 
			}
		
		plane_update = false;
		}

		boost::this_thread::sleep (boost::posix_time::milliseconds(100));
	}
	       
}
 

