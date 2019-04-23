#pragma once

#include "Node.h"

template <typename T>
class MyList
{

private:
	int size;
	Node<T> * firstNode;
	Node<T> * lastNode;
	Node<T> * currentNode;
	int currentIndex ;


public:
	MyList()
	{
		firstNode = NULL;
		lastNode = NULL;
		size = 0 ;
		currentIndex = -1 ;
	}

	int Size()
	{
		return size;
	}

	void Add ( T * e)
	{
		if ( firstNode == NULL )
		{
			firstNode = new Node<T>();
			firstNode->Value = e;
			lastNode = firstNode;
			currentIndex = 0;
			currentNode = firstNode;

		}
		else
		{   
			auto newNode = new Node<T>();
			newNode->BackNode = lastNode;
			lastNode->NextNode = newNode;
			lastNode = newNode;
			lastNode->Value = e;
			
		}
		size++;
	}

	void AddOtherList ( MyList<T> * otherList )
	{
		if ( size > 0 )
		{
			lastNode->NextNode = otherList->firstNode;
			lastNode = otherList ->lastNode;
			size += otherList->size;
			//currentIndex = size - 1;
			//currentNode = lastNode ;
		}

		else
		{
			firstNode = otherList->firstNode;
			lastNode = otherList ->lastNode;
			size = otherList->size;
			//currentIndex = size - 1;
			//currentNode = lastNode ;
		}
	}

	T * At(int index )
	{
		MoveToIndex(index);
		return currentNode->Value;
	}

	T * SetAt(T * value , int index )
	{
		MoveToIndex(index);
		return currentNode->Value = value;
	}

	void RemoveAt ( int index)
	{
		MoveToIndex (index);
		
		if( index == 0 && index == size - 1 )
		{
			firstNode = NULL;
			lastNode = NULL;
			currentNode = NULL;
			currentIndex = -1;
		}
		else if (index == 0)
		{		

			firstNode = firstNode->NextNode;
			firstNode->BackNode = NULL;
			currentNode = firstNode;
			currentIndex = 0;
		}
		else if ( index == size - 1)
		{
			lastNode = lastNode->BackNode;
			lastNode->NextNode = NULL;
			currentNode = lastNode;
			currentIndex = size-2;
		}

		else 
		{
			currentNode->NextNode->BackNode = currentNode->BackNode ;
			currentNode->BackNode->NextNode = currentNode->NextNode ;
			currentNode = currentNode->NextNode;
			currentIndex = index;
		}
		
		size--;
	}

	void MoveToIndex(int index)
	{
		if ( index == currentIndex)
			return;
		if ( index < currentIndex)
		{
			if ( (currentIndex - index) < ( index )  )
			{
				while ( currentIndex != index )
				{
					currentNode = currentNode->BackNode;
					currentIndex --;
				}
			}
			else
			{
				currentNode = firstNode;
				currentIndex = 0 ;
				while ( currentIndex != index )
				{
					currentNode = currentNode->NextNode;
					currentIndex ++;
				}
			}
		}
		else 
		{
			if ( (index - currentIndex ) < ( size - 1 - index )  )
			{
				while ( currentIndex != index )
				{
					currentNode = currentNode->NextNode;
					currentIndex ++;
				}
			}
			else
			{
				currentNode = lastNode;
				currentIndex = size - 1 ;
				while ( currentIndex != index )
				{
					currentNode = currentNode->BackNode;
					currentIndex -- ;
				}
			}
		}
	}

	T ** ToArray()
	{
		T  ** arrayItem = new T*[size] ;
		for ( int i = 0 ; i < size ; i++)
		{
			arrayItem[i] = At(i);
		}

		return arrayItem;
	}

	bool AllDifferents ()
	{
		T  ** arrayItem = new T*[size] ;
		for ( int i = 0 ; i < size ; i++)
		{
			arrayItem[i] = At(i);
		}

		for ( int i = 0 ; i < size ; i++)
		{
			for ( int j = i+1 ; j < size ; j++)
			{
					if ( arrayItem[i] == arrayItem[j] )
						return false;
			}
		}

		return true;
	}			
};