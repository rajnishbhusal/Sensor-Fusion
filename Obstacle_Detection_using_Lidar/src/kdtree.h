/* \author Rajnish Bhusal */
// Implementing kd tree for Clustering

#include "render/render.h"


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

	void insertHelper(Node** node, uint depth, std::vector<float> point, int id){
		if (*node == NULL){
			*node = new Node(point, id);
		}
		else{
			// Calculate current depth
			uint cd = depth%2;

			if(point[cd] < ((*node)->point[cd])){
				insertHelper(&((*node)->left), depth+1, point, id);
			}
			else{
				insertHelper(&((*node)->right), depth+1, point, id);
			}
		
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// Function to insert a new point into the tree
		// the function creates a new node and place correctly with in the root 

		insertHelper(&root, 0, point, id);

	}

	void searchHelper(Node* node, std::vector<float> target, float distanceTol, uint depth, std::vector<int>& ids){
		uint splitAxis = depth % 2;
		
		// 
		if (node!=NULL){
			// box check (from x and y values)
			if (abs(target[0] - node->point[0])<distanceTol && abs(target[1] - node->point[1])<distanceTol && abs(target[2] - node->point[2])<distanceTol){
				// if box check true -> circle check (distance calculation)
				float d;
				d = sqrt((target[0]-node->point[0])*(target[0]-node->point[0]) + (target[1]-node->point[1])*(target[1]-node->point[1]) + (target[2]-node->point[2])*(target[2]-node->point[2]));
				if (d<=distanceTol){
					ids.push_back(node->id);
				}
			}
			if (target[splitAxis]-distanceTol < node->point[splitAxis]){
				searchHelper(node->left, target, distanceTol, depth+1, ids);
			}
			if (target[splitAxis]+distanceTol > node->point[splitAxis]){
				searchHelper(node->right, target, distanceTol, depth+1, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		// to do
		searchHelper(root, target, distanceTol, 0, ids);

		return ids;
	}
	

};




