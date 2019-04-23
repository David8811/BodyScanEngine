#pragma once

template <typename T>
class Node
{
public:
	Node()
	{
		BackNode = NULL;
		NextNode = NULL;
	}
	Node<T> * BackNode;
	Node<T> * NextNode;
	T * Value;
};
