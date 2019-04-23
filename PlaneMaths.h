#pragma once

#include <pcl/common/common.h>
#include "MyList.h"
#include "Plane.h"
#include "GroupsIndexes.h"
#include "PointsDistance.h"
//#include "GPUMath.h"
//#include <cuda_runtime.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

class PlaneMaths
{

private:
//	GPUMath * MyGPUMath;

public:
	PlaneMaths(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> model_cloud);
	~PlaneMaths(void);

	void ChangeColor(MyList<pcl::PointXYZRGBA> points, pcl::RGB rgb)
{
	for( int i=0;i<points.Size();i++)
	{
		points.At(i)->a = rgb.a;
		points.At(i)->r = rgb.r;
		points.At(i)->g = rgb.g;
		points.At(i)->b = rgb.b;
	}


}

void ChangeColor(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud , pcl::RGB rgb)
{
	
	for( int i=0;i<cloud->points.size();i++)
	{
		(&cloud->points[i])->a = rgb.a;
		(&cloud->points[i])->r = rgb.r;
		(&cloud->points[i])->g = rgb.g;
		(&cloud->points[i])->b = rgb.b;
	}
}





// Algoritmos de discriminación de los puntos 

std::vector<pcl::PointXYZRGBA > ObtainingPlanePoints(Plane plane, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float epsilon)
{	
	std::vector< pcl::PointXYZRGBA > result;
	//std::list<pcl::PointXYZRGBA > result;
	
	
	for(int i=0; i < cloud->size() ; i++)
	{
		if ( plane.Contains(cloud->points[i],epsilon) )
		result.push_back(cloud->points[i]);
	}

	return result;
}
/// Esto es para optimizar , pero es equivalente al método anterior 
MyList<pcl::PointXYZRGBA > ObtainingPlanePointsWithMyList(Plane plane, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,float epsilon)
{	
	MyList < pcl::PointXYZRGBA > result;
	//std::list<pcl::PointXYZRGBA > result;
	
	
	for(int i=0; i < cloud->size() ; i++)
	{
		if ( plane.Contains(cloud->points[i],epsilon) )
			result.Add (&(cloud->points[i]));
	}

	return result;
}

///Plano a partir de dos puntos y por defecto de la Normal (0,1,0)
Plane ObtainPlaneByNormal010and2Points(pcl::PointXYZRGBA point1,pcl::PointXYZRGBA point2)
{ 
		Plane result;
		result.Normal.X = 0; result.Normal.Y = 1; result.Normal.Z = 0;
		result.ReferencePoint.x = point1.x;
		result.ReferencePoint.z = point1.z;
		result.ReferencePoint.y = (point1.y + point2.y )/2;
		//El punto generado es un punto ficticio . 

		return   result;
}



 void Swap(double **matrix , int i)
{	
	for ( int k = i+1 ; k < 3; k ++)
	{
		if ( matrix[k][i] !=0 )
		{
			/// Swap rows 
			for ( int m = 0 ; m < 4 ; m ++ )
			{
				double aux ;
				aux = matrix[i][m] ;
				matrix [i][m] = matrix [k][m];
				matrix [k][m] = aux;
			}

			break;
		}
	}		
}

 Vector   GaussEquivalent ( float ** matrix )
 {
	Vector v;
	return v;
 }

 double GaussError ( double ** matrix , Vector vector )
 {
	 double variables[3] = {vector.X , vector.Y , vector.Z}; 
	 double errors[3];
	 errors[0] = matrix[0][3];errors[1] = matrix[1][3];errors[2] =  matrix[2][3];
	 for ( int i = 0 ; i < 3 ; i ++ )
		 for (int j = 0 ; j < 3 ; j ++)
			 errors[i] -= matrix[i][j]*variables[j]; 

	 return abs(errors[0]) + abs(errors[1]) + abs(errors[2]) ;
 }

 Vector   Gauss ( double ** matrix )
{
	/// Esti es para el error para tener la matrix original , no ninguna de sus transformadas 
	double ** matrixOrigin = new double*[3];
	matrixOrigin[0] = new double [4];matrixOrigin[1] = new double [4];matrixOrigin[2] = new double [4];
	for(int i = 0 ; i < 3; i ++ )
		for ( int j= 0 ; j < 4 ; j ++ )
			matrixOrigin[i][j] = matrix[i][j];

	///////////////////////////////////////////////////////////////
	

	for( int i = 0 ; i < 3;i ++)
	{
		//// Por si no tiene coeficiente diferente de cero la i-esima fila con la i-esima columna
		if ( matrix[i][i] == 0 ) 
		{
			Swap(matrix,i);
		}

		for ( int j = i+1;j<3;j++)
		{
			double numFactor = matrix[j][i]  ;
			double denFactor =  matrix[i][i] ;
			for( int k = i ; k < 4 ; k++)
			{
				matrix[j][k] -= (matrix[i][k] * numFactor) / denFactor ; 
			}
		}		
	}	  

	Vector v ;
	v.Z = matrix[2][3] /matrix[2][2];
	v.Y = (matrix[1][3] - (matrix[1][2]* v.Z)) / matrix[1][1]; 
	v.X = ( matrix[0][3] - matrix[0][2]* v.Z - matrix[0][1] *v.Y )/ matrix[0][0] ;
		
	//double error = GaussError(matrixOrigin,v);

	return v;
}

 

Vector ObtainNormalVectorTo3PointsPlane(pcl::PointXYZRGBA point1,pcl::PointXYZRGBA point2,pcl::PointXYZRGBA point3)
{	 
	double N1X,N2X,N3X,N1Y,N2Y,N3Y,N1Z,N2Z,N3Z;
	int step = 0;
	
	/// Mi vector Normal va a tener como comoponente x , Nx - P1x , como y , Ny - P1y , como z , Nz - P1z . Osea es un vector que pasa por el punto P1 
	/// y para que el sistema de ecuaciones tenga una soluci'on , no trivial . 

	/*N1X = point1.x - point2.x;
	N2X = point1.x - point3.x;
	N3X = point2.x - point3.x;

	N1Y = point1.y - point2.y;
	N2Y = point1.y - point3.y;
	N3Y = point2.y - point3.y;

	N1Z = point1.z - point2.z;
	N2Z = point1.z - point3.z;
	N3Z = point2.z - point3.z;*/

	N1X = ((double)point1.x) - ((double)point2.x);
	N2X = ((double)point1.x) - ((double)point3.x);
	N3X = ((double)point2.x) - ((double)point3.x);

	N1Y = ((double)point1.y) - ((double)point2.y);
	N2Y = ((double)point1.y) - ((double)point3.y);
	N3Y = ((double)point2.y) - ((double)point3.y);

	N1Z = ((double)point1.z) - ((double)point2.z);
	N2Z = ((double)point1.z) - ((double)point3.z);
	N3Z = ((double)point2.z) - ((double)point3.z);

	double ** matrix = new double*[3] ;
	matrix[0] = new double[4];
	matrix[1] = new double[4];
	matrix[2] = new double[4];
	

	matrix [0][0] = N1X;matrix [0][1] = N1Y;matrix [0][2] = N1Z;
	matrix [1][0] = N2X;matrix [1][1] = N2Y;matrix [1][2] = N2Z;
	matrix [2][0] = N3X;matrix [2][1] = N3Y;matrix [2][2] = N3Z;

	// Estos es el vector B , de AX = B 
	/*matrix[0][3] = point1.x * ( point1.x - point2.x) + point1.y * ( point1.y - point2.y) + point1.z * ( point1.z - point2.z) ; 
	matrix[1][3] = point1.x * ( point1.x - point3.x) + point1.y * ( point1.y - point3.y) + point1.z * ( point1.z - point3.z) ;
	matrix[2][3] = point1.x * ( point2.x - point3.x) + point1.y * ( point2.y - point3.y) + point1.z * ( point2.z - point3.z) ; */

	matrix[0][3] = ((double)point1.x )* ((double)( point1.x - point2.x)) + ((double)point1.y )* ((double)( point1.y - point2.y)) + ((double)point1.z ) * ((double) (point1.z - point2.z)) ; 
	matrix[1][3] = ((double)point1.x )* ((double)( point1.x - point3.x)) + ((double)point1.y )* ((double)( point1.y - point3.y)) + ((double)point1.z ) * ((double)( point1.z - point3.z)) ;
	matrix[2][3] = ((double)point1.x )* ((double)( point2.x - point3.x)) + ((double)point1.y )* ((double)( point2.y - point3.y)) + ((double)point1.z ) * ((double)( point2.z - point3.z)) ; 

	//return GaussEquivalent(matrix);
	Vector vector =  Gauss(matrix);
	vector .X -= point1.x;
	vector .Y -= point1.y;
	vector .Z -= point1.z;

	return vector; 


}


///Plano a partir de dos puntos y por defecto de la Normal (0,1,0)
Plane ObtainPlaneBy3Points(pcl::PointXYZRGBA point1,pcl::PointXYZRGBA point2,pcl::PointXYZRGBA point3)
{ 
		Plane result;
		result.Normal.X = 0; result.Normal.Y = 1; result.Normal.Z = 0; 
		result.Normal = ObtainNormalVectorTo3PointsPlane(point1 ,point2,point3);
		result.ReferencePoint.x = point1.x;
		result.ReferencePoint.z = point1.z;
		result.ReferencePoint.y = point1.y ; 

		return   result;
}





///Plano a partir de un punto y por defecto de la Normal (0,1,0)
Plane ObtainPlaneByNormal010and1Point(pcl::PointXYZRGBA point1,float angle = 0)
{ 
		Plane result;
		result.Normal.X = 0; result.Normal.Y = 1; result.Normal.Z = tan(angle*3.141516/180);
		result.ReferencePoint.x = point1.x;
		result.ReferencePoint.z = point1.z;
		result.ReferencePoint.y = point1.y ;
		//El punto generado es un punto ficticio . 

		return   result;
}



// Este devuelve solo los puntos que pertenecen al plano , quitandole los que pertenecen a los brazos
// basado en que se conoce un punto includingPoint que pertenece a este conjunto resultante .
// esto sirve para quedarnos con el valor de la cintura , del pecho y del cuello .
// Asumo que el nearEpsilon se tome en metros 
std::vector< pcl::PointXYZRGBA > ObtainingBodyPlanePointsByOLD(std::vector<pcl::PointXYZRGBA > planePoints, pcl::PointXYZRGBA includingPoint, float nearEpsilon )
{	
	std::vector< pcl::PointXYZRGBA > result;
	result.push_back(includingPoint);
	int count = planePoints.size () ;
	bool* isNotInResult = new bool[planePoints.size ()] ;

	for ( int i = 0 ; i < planePoints.size () ; i++)
	{
		for ( int j = 0 ; j < result.size() ; j++)
		{
		// si existe alguien meterlo y empezar de nuevo para buscar m'as con el nuevo conjunto
			if (  isNotInResult[i] && Utils::PointDistance (result[j],planePoints[i])  < nearEpsilon )			
			{
				result.push_back(planePoints[i]);
				isNotInResult [i] = false ;
				// esto es para empezar de nuevo . 
				i =  0;
				j = -1;
			}
		}		
	}	
	delete isNotInResult; 
	return result ;
}

// Este devuelve solo los puntos que pertenecen al plano , quitandole los que pertenecen a los brazos
// basado en que se conoce un punto includingPoint que pertenece a este conjunto resultante .
// esto sirve para quedarnos con el valor de la cintura , del pecho y del cuello .
// Asumo que el nearEpsilon se tome en metros 
MyList<pcl::PointXYZRGBA> ObtainingBodyPlanePointsBy(std::vector<pcl::PointXYZRGBA > planePoints, pcl::PointXYZRGBA includingPoint, float nearEpsilon )
{	
	MyList<pcl::PointXYZRGBA> setresult;
	MyList<pcl::PointXYZRGBA> searcheds;
	MyList<pcl::PointXYZRGBA> setPlanePoints;
	float nearEpsilonPower2 =  nearEpsilon * nearEpsilon;
	
	for (int i = 0 ; i < planePoints.size() ; i++ )
	{
		setPlanePoints.Add(& (planePoints[i])) ;
	}

	/// Buscando los m'as cercas de including points que asumo que no van a estar en nearepsilon por ser muy pequeño 
	for ( int i = 0 ; i < setPlanePoints.Size();i++)
	{
		// si existe alguien meterlo y empezar de nuevo para buscar m'as con el nuevo conjunto
		//	if ( PointDistance (includingPoint,*setPlanePoints.At(i))  < nearEpsilon )	
			if ( Utils::PointDistancePower2 (&includingPoint,setPlanePoints.At(i))  < nearEpsilonPower2 )	
			{
				searcheds.Add(setPlanePoints.At(i));
				setPlanePoints.RemoveAt(i);
				i--;
			}	
	}

	while ( searcheds .Size() > 0 )
	{
			for ( int i = 0 ; i < setPlanePoints.Size();i++)
		{
			// si existe alguien meterlo y empezar de nuevo para buscar m'as con el nuevo conjunto
			//	if ( PointDistance (searcheds.At(0),setPlanePoints.At(i))  < nearEpsilon )	
				if ( Utils::PointDistancePower2 (searcheds.At(0),setPlanePoints.At(i))  < nearEpsilonPower2 )
				{
					searcheds.Add(setPlanePoints.At( i))  ;	
					setPlanePoints.RemoveAt(i);
					i--;
				}
		}
			setresult.Add(searcheds.At(0));
			searcheds.RemoveAt(0) ;
	}

	return setresult;
}

GroupsIndexes ObtainGroupsIndexes( MyList<MyList<pcl::PointXYZRGBA>> * groups , PointsDistance pointsDistance )
{
	MyList<pcl::PointXYZRGBA> points ;
	int  * indexes = new int[2] ; 
	indexes[0] = -1;
	indexes[1] = -2;
	int indexesIndex = 0;

	int i = 0;
	int j = 0;

	for (  ; i < (groups)->Size() ; i ++ )
	{
		for (  ; j < ((groups)->At(i))->Size() ; j ++ )
		{
			if ( pointsDistance.point1 == ((groups)->At(i))->At(j) )
			{
				points.Add(pointsDistance.point1);
				indexes[indexesIndex] = i;
				indexesIndex++;
				i = (groups)->Size() ;
				break;
			}
		}
	}

	pcl::PointXYZRGBA * groupPoint;
	
		i = 0;
		j = 0;
	for (  ; i < (groups)->Size() ; i ++ )
	{
		for ( ; j < ((groups)->At(i))->Size() ; j ++ )
		{
			groupPoint = ((groups)->At(i))->At(j);
			if ( pointsDistance.point2 == groupPoint )
			{
				points.Add(pointsDistance.point2);
				indexes[indexesIndex] = i;
				indexesIndex++;

				// Borrar Quitar
				//goto salto;
				// 

				i = (groups)->Size() ;
				break;
			}
		}
	}
// Borrar Quitar 	
//salto:
/// 

	if ( points.Size() == 2 )
	{
		return GroupsIndexes(indexes,2);
	}

	else if ( points.Size() == 1 )
	{
		return GroupsIndexes(indexes[0],points.At(0)) ;
	}

	return GroupsIndexes();
}

void MergeGroups(int index1,int index2 , MyList<MyList<pcl::PointXYZRGBA>>     *    groups)
{
	if ( index1 > index2)
	{
		int aux = index1;
		index1 = index2;
		index2 = aux;
	} 

	auto firstGroup = (groups)->At(index1) ;
	auto secondGroup = (groups)->At(index2);
	firstGroup->AddOtherList(secondGroup);
	groups -> RemoveAt(index2) ;
}

MyList<pcl::PointXYZRGBA> ObtainingBodyPlanePointsByWithMyListByBigGroup(MyList<pcl::PointXYZRGBA > planePoints, float nearEpsilon )
{
	MyList<pcl::PointXYZRGBA> setresult;
	
	int  * pointsGroups = new int[planePoints.Size()];
	int groupCount = 0;
	MyList<MyList<int>> groupsPointsIndexes ;
	auto setPlanePoints = planePoints.ToArray();
	//int pointsDistanceSize = ( planePoints.Size() * ( planePoints.Size() - 1) ) / 2 ; 
	//PointsDistance  * pointsDistance = new  PointsDistance [ pointsDistanceSize  ] ;

	for (int i = 0 ; i < planePoints.Size() ; i ++ )
	{
		/// cada punto no tiene grupo al principio 
		pointsGroups[i] = -1;
	}

	for ( int i = 0 ; i < planePoints.Size() ; i ++ )
	{
		for ( int j = i + 1 ; j < planePoints.Size() ; j ++ )
		{
			if ( pointsGroups[i] != -1 )
			{
					if ( pointsGroups[i] != pointsGroups[j] ) // sino est'an en el mismo grupo hasta el momento 
				{
					float distance = (Utils::PointDistance(setPlanePoints[i],setPlanePoints[j]));

					if ( distance < nearEpsilon )
					{

						if ( pointsGroups [j] == -1 ) 
						{
							pointsGroups [j] = pointsGroups[i];
							groupsPointsIndexes.At (pointsGroups [i])->Add( new int(j) );
						}
						else
						{
							int deadGroupIndex = pointsGroups[j] ;
							auto deadGroup = groupsPointsIndexes.At(deadGroupIndex);
							for ( int k = 0 ; k < deadGroup->Size() ; k++)
							{
								pointsGroups[*(deadGroup->At(k))] = pointsGroups[i];
							}
							auto liveGroup = groupsPointsIndexes.At(pointsGroups[i]);
							liveGroup->AddOtherList(deadGroup);
							groupsPointsIndexes.SetAt(NULL,deadGroupIndex);
						}
					}
				}
			}
			else if ( pointsGroups [j] != -1 )
			{
				// es obvio que pointsGroups[i] == -1 
				float distance = (Utils::PointDistance(setPlanePoints[i],setPlanePoints[j]));
				if ( distance < nearEpsilon )
				{
							pointsGroups [i] = pointsGroups[j];
							groupsPointsIndexes.At (pointsGroups [j])->Add( new int(i) );
				}
			}

			else 
			{
				float distance = (Utils::PointDistance(setPlanePoints[i],setPlanePoints[j]));
				if ( distance < nearEpsilon )
				{
					pointsGroups [i] = groupCount;
					pointsGroups [j] = groupCount;
					MyList<int> * newGroupIndexes = new MyList<int>;
					newGroupIndexes->Add(new int(i));
					newGroupIndexes->Add(new int(j));
					groupsPointsIndexes.Add(newGroupIndexes);
					groupCount++;
				}
			}
		}
	}
	int indexMax = -1 ;
	int countMax = 0;

	for ( int i = 0 ; i<groupsPointsIndexes.Size() ; i++ )
	{
		if ( groupsPointsIndexes.At (i) != NULL && groupsPointsIndexes.At(i)->Size() > countMax )
		{
			indexMax = i ;
			countMax = groupsPointsIndexes.At(i)->Size() ;
		}
	}

	for ( int i = 0 ; i < groupsPointsIndexes.At(indexMax)->Size() ; i ++ )
	{
		setresult.Add( setPlanePoints[ *(groupsPointsIndexes.At(indexMax) ->At(i) )] )  ;
	}
	
	return setresult;

}
MyList<pcl::PointXYZRGBA> ObtainingBodyPlanePointsByWithMyListByBigGroupV1(MyList<pcl::PointXYZRGBA > planePoints, float nearEpsilon )
{
	MyList<pcl::PointXYZRGBA> setresult;
	
	float nearEpsilon2 = nearEpsilon * nearEpsilon; 
	int  * pointsGroups = new int[planePoints.Size()];
	int groupCount = 0;
	MyList<MyList<int>> groupsPointsIndexes ;
	auto setPlanePoints = planePoints.ToArray();
	//int pointsDistanceSize = ( planePoints.Size() * ( planePoints.Size() - 1) ) / 2 ; 
	//PointsDistance  * pointsDistance = new  PointsDistance [ pointsDistanceSize  ] ;

	for (int i = 0 ; i < planePoints.Size() ; i ++ )
	{
		/// cada punto no tiene grupo al principio 
		pointsGroups[i] = -1;
	}

	for ( int i = 0 ; i < planePoints.Size() ; i ++ )
	{
		for ( int j = i + 1 ; j < planePoints.Size() ; j ++ )
		{
			if ( pointsGroups[i] != -1 )
			{
					if ( pointsGroups[i] != pointsGroups[j] ) // sino est'an en el mismo grupo hasta el momento 
				{
					float distancesqrt2 = (Utils::PointDistancePower2(setPlanePoints[i],setPlanePoints[j]));
					if ( distancesqrt2 < nearEpsilon2 )
					{

						if ( pointsGroups [j] == -1 ) 
						{
							pointsGroups [j] = pointsGroups[i];
							groupsPointsIndexes.At (pointsGroups [i])->Add( new int(j) );
						}
						else
						{
							int deadGroupIndex = pointsGroups[j] ;
							auto deadGroup = groupsPointsIndexes.At(deadGroupIndex);
							for ( int k = 0 ; k < deadGroup->Size() ; k++)
							{
								pointsGroups[*(deadGroup->At(k))] = pointsGroups[i];
							}
							auto liveGroup = groupsPointsIndexes.At(pointsGroups[i]);
							liveGroup->AddOtherList(deadGroup);
							groupsPointsIndexes.SetAt(NULL,deadGroupIndex);
						}
					}
				}
			}
			else if ( pointsGroups [j] != -1 )
			{
				// es obvio que pointsGroups[i] == -1 
				float distancesqrt2 = (Utils::PointDistancePower2(setPlanePoints[i],setPlanePoints[j]));
				if ( distancesqrt2 < nearEpsilon2 )
					{
							pointsGroups [i] = pointsGroups[j];
							groupsPointsIndexes.At (pointsGroups [j])->Add( new int(i) );
					}
			}

			else 
			{
				float distancesqrt2 = (Utils::PointDistancePower2(setPlanePoints[i],setPlanePoints[j]));
				if ( distancesqrt2 < nearEpsilon2 )
					{
					pointsGroups [i] = groupCount;
					pointsGroups [j] = groupCount;
					MyList<int> * newGroupIndexes = new MyList<int>;
					newGroupIndexes->Add(new int(i));
					newGroupIndexes->Add(new int(j));
					groupsPointsIndexes.Add(newGroupIndexes);
					groupCount++;
					}
			}
		}
	}
	int indexMax = -1 ;
	int countMax = 0;

	for ( int i = 0 ; i<groupsPointsIndexes.Size() ; i++ )
	{
		if ( groupsPointsIndexes.At (i) != NULL && groupsPointsIndexes.At(i)->Size() > countMax )
		{
			indexMax = i ;
			countMax = groupsPointsIndexes.At(i)->Size() ;
		}
	}

	for ( int i = 0 ; i < groupsPointsIndexes.At(indexMax)->Size() ; i ++ )
	{
		setresult.Add( setPlanePoints[ *(groupsPointsIndexes.At(indexMax) ->At(i) )] )  ;
	}
	
	return setresult;

}


MyList<pcl::PointXYZRGBA> DavidObtainingBodyPlanePointsByWithMyList(MyList<pcl::PointXYZRGBA > planePoints, pcl::PointXYZRGBA includingPoint, float nearEpsilon )
{




return planePoints;
}


float GetXDirectorCosine(pcl::PointXYZRGBA p1, pcl::PointXYZRGBA p2)
{

	float x = p2.x - p1.x;
	float y = p2.y - p1.y;
	float z = p2.z - p1.z;

	float norm = sqrt(x*x+y*y+z*z);

	return cos(x/norm);
}

//
MyList<pcl::PointXYZRGBA> GPUObtainingBodyPlanePointsByWithMyList(MyList<pcl::PointXYZRGBA > planePoints, float * CutPoints)
{	

	

/*	IplImage * MyImage = cvCreateImage(cvSize(1024,768),IPL_DEPTH_8U, 3);*/
	MyList<pcl::PointXYZRGBA> result;
	

	
		for(int i = 0; i < planePoints.Size();i++)
	{
	
		if(CutPoints[0] != -1000 && CutPoints[1] != -1000)
		{
			if(planePoints.At(i)->x > CutPoints[0] && planePoints.At(i)->x < CutPoints[1])
			{
				
				result.Add(planePoints.At(i));
				
			}
		}
		else
			result.Add(planePoints.At(i));
	}


/*		for(int i = 0; i < result.Size();i++)
	{
		int x = 1024/2+result.At(i)->x*100;
		int y = 768/2+(1-result.At(i)->z)*100;
		cvDrawCircle(MyImage,cvPoint(x,y),2,cvScalar(255,0,0));
	}

	
	
	cvShowImage("Prueba",MyImage);
	cvWaitKey(0);
	*/
//	angle = acos(GetXDirectorCosine(includingPoint,*setresult.At(setresult.Size()-1)))*180/M_PI;*/

	return result;
}



// Este devuelve solo los puntos que pertenecen al plano , quitandole los que pertenecen a los brazos
// basado en que se conoce un punto includingPoint que pertenece a este conjunto resultante .
// esto sirve para quedarnos con el valor de la cintura , del pecho y del cuello .
// Asumo que el nearEpsilon se tome en metros 
MyList<pcl::PointXYZRGBA> ObtainingBodyPlanePointsByWithMyList(MyList<pcl::PointXYZRGBA > planePoints, pcl::PointXYZRGBA includingPoint, float nearEpsilon )
{	
	MyList<pcl::PointXYZRGBA> setresult;
	MyList<pcl::PointXYZRGBA> searcheds;
//	MyList<pcl::PointXYZRGBA> setPlanePoints;
	float nearEpsilonPower2 =  nearEpsilon * nearEpsilon;
	
	/// Esto lo tengo puesto para no afectar el planePoints inicial , pero se puede para optimizar , simplemente trabajar con el inicial sabiendo que puede cambiar 
/*	for (int i = 0 ; i < planePoints.Size() ; i++ )
	{
		setPlanePoints.Add((planePoints.At(i))) ;
	}*/
	/// 

	/// Buscando los más cercas de including points que asumo que no van a estar en nearepsilon por ser muy pequeño 
	for ( int i = 0 ; i < planePoints.Size();i++)
	{
		// si existe alguien meterlo y empezar de nuevo para buscar m'as con el nuevo conjunto
			
			if ( Utils::PointDistancePower2 (&includingPoint,planePoints.At(i))  < nearEpsilonPower2 )	
			{
				searcheds.Add(planePoints.At(i));
				planePoints.RemoveAt(i);
				i--;
			}	
	}


	while ( searcheds .Size() > 0 )
	{
			for ( int i = 0 ; i < planePoints.Size();i++)
		{
			// si existe alguien meterlo y empezar de nuevo para buscar m'as con el nuevo conjunto
				
				if ( Utils::PointDistancePower2 (searcheds.At(0),planePoints.At(i))  < nearEpsilonPower2 )
				{
					searcheds.Add(planePoints.At( i))  ;	
					planePoints.RemoveAt(i);
					i--;
				}
		}
			setresult.Add(searcheds.At(0));
			searcheds.RemoveAt(0) ;
	}

	

	return setresult;
}

pcl::PointXYZRGBA NearFrom(MyList<pcl::PointXYZRGBA> planePoints, pcl::PointXYZRGBA point)
{
	float nearDistance = 100000000;
	pcl::PointXYZRGBA * nearPoint = &point ;

	for	(int i = 0 ; i < planePoints.Size() ; i ++ )
	{
		auto actualDistance = Utils::PointDistancePower2 ( &point , planePoints.At(i) );
		if (  actualDistance < nearDistance )
		{
			nearPoint = planePoints.At(i);
			nearDistance = actualDistance;
		}
	}

	return *nearPoint ;
}

pcl::PointXYZRGBA NearFromOptimize(MyList<pcl::PointXYZRGBA> planePoints, pcl::PointXYZRGBA point)
{
	float nearDistance = 100000000;
	pcl::PointXYZRGBA * nearPoint = &point ;

	for	(int i = 0 ; i < planePoints.Size() ; i ++ )
	{
		auto actualDistance = Utils::PointDistancePower2 ( &point , planePoints.At(i) );
		if (  actualDistance < nearDistance )
		{
			nearPoint = planePoints.At(i);
			nearDistance = actualDistance;
		}
	}

	return *nearPoint ;
}

////////////////////////////////////////////////// MÉTODOS OBJETIVOS  //////////////////////////////////////////////////////////////////////////////////////////////////////


//// Estos son las variantes de métodos objetivos o finales que devuelven resultados con los mismos fines que es el de obtener solo los puntos que rodean a la cintura
/// el pecho , el cuello , etc .

/// este método obtiene el plano a partir de un punto ficticio formado a partir de dos puntos del cloud_point, escogidos por el usuario, que se crea como promedio de estos dos últimos
//boost::container::flat_set<pcl::PointXYZRGBA > ObtainingBodyPlanePoints( pcl::PointXYZRGBA point1 , pcl::PointXYZRGBA point2 ,  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud )
MyList<pcl::PointXYZRGBA> ObtainingBodyPlanePointsUser2points( pcl::PointXYZRGBA point1 , pcl::PointXYZRGBA point2 ,  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud )
{
	//////////////////////
	Plane plane = ObtainPlaneByNormal010and2Points (point1,point2);
	//////////////////////

	//////////////////////
	//auto planePoints = ObtainingPlanePoints(plane,cloud,0.001);
	auto planePoints = ObtainingPlanePointsWithMyList(plane,cloud,0.002);
	//////////////////////

	//////////////////////
	// Asumo que el mismo punto con que se define el Plane por el m'etodo ObtainPlaneByNormal010and2Points que lo hace por el promedio de las Ys de los puntos 
	// asumo que es un candidato ficticio a formar parte del conjunto . Se podría pensar en otro . 
	
	///este presenta problemas cuando estamos por el pecho debido a que el plane.ReferencePoint es un punto ficticio , en la variante V1 se busca un punto pero se vuelve más
	/// costoso 
	return ObtainingBodyPlanePointsByWithMyList ( planePoints , plane.ReferencePoint, 0.004 )  ; 
	//////////////////////
}

/// este método obtiene el plano a partir de un punto ficticio formado a partir de dos puntos del cloud_point, escogidos por el usuario, que se crea como promedio de estos dos últimos
/// esta variante busca sustituir el punto ficticio plane.ReferencePoint por uno real que sea el m'as cercano a este que pertenezca al plano . 
//boost::container::flat_set<pcl::PointXYZRGBA > ObtainingBodyPlanePoints( pcl::PointXYZRGBA point1 , pcl::PointXYZRGBA point2 ,  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud )
MyList<pcl::PointXYZRGBA> ObtainingBodyPlanePointsUser2pointsV1( pcl::PointXYZRGBA point1 , pcl::PointXYZRGBA point2 ,  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud )
{
	//////////////////////
	Plane plane = ObtainPlaneByNormal010and2Points (point1,point2);
	//////////////////////

	//////////////////////
	//auto planePoints = ObtainingPlanePoints(plane,cloud,0.001);
	auto planePoints = ObtainingPlanePointsWithMyList(plane,cloud,0.001);
	//////////////////////

	//////////////////////
	// Asumo que el mismo punto con que se define el Plane por el m'etodo ObtainPlaneByNormal010and2Points que lo hace por el promedio de las Ys de los puntos 
	// asumo que es un candidato ficticio a formar parte del conjunto . Se podría pensar en otro . 
	

	return ObtainingBodyPlanePointsByWithMyList ( planePoints , NearFrom(planePoints,plane.ReferencePoint), 0.005 )  ; 
	//////////////////////
}

/// este método obtiene el plano a partir de un punto ficticio formado a partir de dos puntos del cloud_point, escogidos por el usuario, que se crea como promedio de estos dos últimos
/// esta variante busca sustituir el punto ficticio plane.ReferencePoint por uno real que sea el m'as cercano a este que pertenezca al plano . 
//boost::container::flat_set<pcl::PointXYZRGBA > ObtainingBodyPlanePoints( pcl::PointXYZRGBA point1 , pcl::PointXYZRGBA point2 ,  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud )
MyList<pcl::PointXYZRGBA> ObtainingBodyPlanePointsUser2pointsV2( pcl::PointXYZRGBA point1 , pcl::PointXYZRGBA point2 ,  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud )
{
	/// Cambio de colores Borrar !!!!!! ///
	pcl::RGB rgb ;

		/*		rgb.r = 0;
				rgb.g = 0;
				rgb.b = 250;
				rgb.a = 128;

				ChangeColor(cloud,rgb);*/
	///////////////////////////////////////////////
				

	//////////////////////
	Plane plane = ObtainPlaneByNormal010and2Points (point1,point2);
	//////////////////////
	
	//////////////////////
	//auto planePoints = ObtainingPlanePoints(plane,cloud,0.001);
	auto planePoints = ObtainingPlanePointsWithMyList(plane,cloud,0.002);
	//////////////////////

	/// Cambio de colores Borrar !!!!!! ///

			/*	rgb.r = 0;
				rgb.g = 250;
				rgb.b = 0;
				rgb.a = 128;
				ChangeColor(planePoints,rgb);*/
	//////////////////////

	//////////////////////
	// Asumo que el mismo punto con que se define el Plane por el m'etodo ObtainPlaneByNormal010and2Points que lo hace por el promedio de las Ys de los puntos 
	// asumo que es un candidato ficticio a formar parte del conjunto . Se podría pensar en otro . 
	
	pcl::PointXYZRGBA pointAverage;
	pointAverage.y = (point1.y + point2.y)/2;
	pointAverage.x = (point1.x + point2.x)/2;
	pointAverage.z = (point1.z + point2.z)/2;

	auto result = ObtainingBodyPlanePointsByWithMyList ( planePoints , NearFrom(planePoints,pointAverage), 0.005 )  ; 

	/// Cambio de colores Borrar !!!!!! ///

		/*		rgb.r = 250;
				rgb.g = 0;
				rgb.b = 0;
				rgb.a = 128;
				ChangeColor(result,rgb);*/
	//////////////////////

	return result;
}

MyList<pcl::PointXYZRGBA> ObtainingBodyPlanePointsUser2pointsByGreatGroup( pcl::PointXYZRGBA point1 , pcl::PointXYZRGBA point2 ,  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud )
{
	/// Cambio de colores Borrar !!!!!! ///
	pcl::RGB rgb ;

	/*			rgb.r = 0;
				rgb.g = 0;
				rgb.b = 250;
				rgb.a = 128;*/
	///////////////////////////////////////////////
				

	//////////////////////
	Plane plane = ObtainPlaneByNormal010and2Points (point1,point2);
	//////////////////////
	
	//////////////////////
	//auto planePoints = ObtainingPlanePoints(plane,cloud,0.001);
	auto planePoints = ObtainingPlanePointsWithMyList(plane,cloud,0.001);
	//////////////////////

	/// Cambio de colores Borrar !!!!!! ///

			/*	rgb.r = 0;
				rgb.g = 250;
				rgb.b = 0;
				rgb.a = 128;
				ChangeColor(planePoints,rgb);*/
	//////////////////////

	//////////////////////
	// Asumo que el mismo punto con que se define el Plane por el m'etodo ObtainPlaneByNormal010and2Points que lo hace por el promedio de las Ys de los puntos 
	// asumo que es un candidato ficticio a formar parte del conjunto . Se podría pensar en otro . 
	
	auto result = ObtainingBodyPlanePointsByWithMyListByBigGroup ( planePoints , 0.005 )  ; 

	/// Cambio de colores Borrar !!!!!! ///

			/*	rgb.r = 250;
				rgb.g = 0;
				rgb.b = 0;
				rgb.a = 128;
				ChangeColor(result,rgb);*/
	//////////////////////

	return result;
}
MyList<pcl::PointXYZRGBA> ObtainingBodyPlanePointsUser2pointsByGreatGroupV1( pcl::PointXYZRGBA point1 , pcl::PointXYZRGBA point2 ,  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud )
{
	/// Cambio de colores Borrar !!!!!! ///
/*	pcl::RGB rgb ;

				rgb.r = 0;
				rgb.g = 0;
				rgb.b = 250;
				rgb.a = 128;*/
	///////////////////////////////////////////////
				

	//////////////////////
	Plane plane = ObtainPlaneByNormal010and2Points (point1,point2);
	//////////////////////
	
	//////////////////////
	//auto planePoints = ObtainingPlanePoints(plane,cloud,0.001);
	auto planePoints = ObtainingPlanePointsWithMyList(plane,cloud,0.001);
	//////////////////////


	//////////////////////
	// Asumo que el mismo punto con que se define el Plane por el m'etodo ObtainPlaneByNormal010and2Points que lo hace por el promedio de las Ys de los puntos 
	// asumo que es un candidato ficticio a formar parte del conjunto . Se podría pensar en otro . 
	
	auto result = ObtainingBodyPlanePointsByWithMyListByBigGroupV1 ( planePoints , 0.005 )  ; 


	return result;
}



/// este método obtiene el plano a partir de un punto real escogido por el usuario
//boost::container::flat_set<pcl::PointXYZRGBA > ObtainingBodyPlanePoints( pcl::PointXYZRGBA point1 , pcl::PointXYZRGBA point2 ,  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud )
MyList<pcl::PointXYZRGBA> ObtainingBodyPlanePointsUser1Point( pcl::PointXYZRGBA point1 ,  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,Plane& resultPlane,float angle  = 0.005,float epsilon = 0, float beta = 0.005)
{
	

	//////////////////////
	Plane plane = ObtainPlaneByNormal010and1Point (point1,angle);
	//////////////////////
	resultPlane = plane;
	//////////////////////

	
	
	auto planePoints = ObtainingPlanePointsWithMyList(plane,cloud,epsilon);

	

//	auto planeGPUPoints = MyGPUMath->GPUObtainPlanePoints(plane.ReferencePoint,angle,epsilon);
	//////////////////////
	
	//////////////////////
	// Asumo que el mismo punto con que se define el Plane por el m'etodo ObtainPlaneByNormal010and2Points que lo hace por el promedio de las Ys de los puntos 
	// asumo que es un candidato ficticio a formar parte del conjunto . Se podría pensar en otro . 
	
  /*  float * CutPoints = new float[2];
    CutPoints =  MyGPUMath->GPUObtainCutPoints(planePoints);*/
//	auto resultGPU = GPUObtainingBodyPlanePointsByWithMyList( planePoints , CutPoints)  ; 

	auto resultCPU = ObtainingBodyPlanePointsByWithMyList ( planePoints , plane.ReferencePoint, beta)  ; 

    
   return resultCPU;


	//////////////////////
}

/// este método obtiene el plano auxiliándose del plano manual de medición
MyList<pcl::PointXYZRGBA> ObtainingBodyPlaneByMeasurementPlane( pcl::PointXYZRGBA point1,float min_x, float max_x,float angle,Plane& resultPlane, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{

	MyList<pcl::PointXYZRGBA> result;
	
	Plane plane = ObtainPlaneByNormal010and1Point (point1,angle);

	resultPlane = plane;

	cout << "Plane angle: " << angle << endl;

	for(int i = 0; i < cloud->size();i++)
		if(cloud->points[i].x >= min_x && cloud->points[i].x <= max_x)			
			if(plane.Contains(cloud->points[i],0.005))
					result.Add(&(cloud->points[i]));
	
	return result;
}

/// 
//boost::container::flat_set<pcl::PointXYZRGBA > ObtainingBodyPlanePoints( pcl::PointXYZRGBA point1 , pcl::PointXYZRGBA point2 ,  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud )
MyList<pcl::PointXYZRGBA> ObtainingBodyPlanePointsUser3points( pcl::PointXYZRGBA point1 , pcl::PointXYZRGBA point2 ,  pcl::PointXYZRGBA point3 ,  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud )
{
	//////////////////////
	Plane plane = ObtainPlaneBy3Points (point1,point2,point3);
	//////////////////////
	bool in1 = plane.Contains(point1,0.001);
	bool in2 = plane.Contains(point2,0.001);
	bool in3 = plane.Contains(point3,0.001);

	//////////////////////
	//auto planePoints = ObtainingPlanePoints(plane,cloud,0.001);
	auto planePoints = ObtainingPlanePointsWithMyList(plane,cloud,0.001);
	//////////////////////

	//////////////////////
	// Asumo que el mismo punto con que se define el Plane por el m'etodo ObtainPlaneByNormal010and2Points que lo hace por el promedio de las Ys de los puntos 
	// asumo que es un candidato ficticio a formar parte del conjunto . Se podría pensar en otro . 
	
	///este presenta problemas cuando estamos por el pecho debido a que el plane.ReferencePoint es un punto ficticio , en la variante V1 se busca un punto pero se vuelve más
	/// costoso 
	return ObtainingBodyPlanePointsByWithMyList ( planePoints , plane.ReferencePoint, 0.005 )  ; 
	//////////////////////
}
};

