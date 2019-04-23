#pragma once

#include <pcl/common/common.h>

class GroupsIndexes
{
public:
	int * Indexes;
	int LenthIndexes;
	/// El point alone es cuando solo hay un punto del PointsDistance correspondiente por supuesto en un solo grupo de puntos ,
	/// y as'i añadir el otro y no tener que calcularlo de nuevo
	pcl::PointXYZRGBA * PointAlone;
	GroupsIndexes(int * pIndexes ,int pLenthIndexes)
	{
		Indexes = pIndexes;
		LenthIndexes = pLenthIndexes;
	}

	GroupsIndexes(int justOneIndex,pcl::PointXYZRGBA * pPointAlone  )
	{
		Indexes = new int[1];
		Indexes[0]  = justOneIndex;
		LenthIndexes = 1;
		PointAlone = pPointAlone ;
	}

	GroupsIndexes()
	{
		LenthIndexes = 0 ;
		PointAlone = NULL;
		Indexes = NULL; 
		PointAlone = NULL;
	}
};