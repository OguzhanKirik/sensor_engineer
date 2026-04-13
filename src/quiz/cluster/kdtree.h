/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

		if(root == NULL){
			root = new Node(point, id);
			return;
		}

		Node* current = root;
		int depth = 0;

		while (current != NULL){
			int axis = depth % 2;

			if (point[axis] < current->point[axis]){
				if (current->left == NULL){
					current->left = new Node(point, id);
					return;
				}
				current = current->left;
			}
			else{
				if (current->right == NULL){
					current->right = new Node(point, id);
					return;
				}
				current = current->right;
			}
			depth++;
		}

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper(target, root, 0, distanceTol, ids);
		
		return ids;


	}
	
	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if (node != NULL)
		{
			if (node->point[0] >= target[0] - distanceTol && node->point[0] <= target[0] + distanceTol &&
				node->point[1] >= target[1] - distanceTol && node->point[1] <= target[1] + distanceTol)
			{
				float dx = node->point[0] - target[0];
				float dy = node->point[1] - target[1];
				float distance = sqrt(dx * dx + dy * dy);

				if (distance <= distanceTol)
					ids.push_back(node->id);
			}

			int axis = depth % 2;

			if (target[axis] - distanceTol < node->point[axis])
				searchHelper(target, node->left, depth + 1, distanceTol, ids);

			if (target[axis] + distanceTol > node->point[axis])
				searchHelper(target, node->right, depth + 1, distanceTol, ids);
		}
	}
};




