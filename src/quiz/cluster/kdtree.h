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
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		if (root == nullptr) {
		    root = new Node(point, id);
		    return;
		}

        insert_recursive(root, point, id, {});
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}

private:
    void insert_recursive(Node* node, std::vector<float> point, int id, int xy)
    {
	    const auto reference_value = node->point[xy % 2];
        const auto point_value = point[xy % 2];

        auto& node_p = (point_value < reference_value) ? node->left : node->right;

        if (node_p == nullptr) {
            node_p = new Node(point, id);
            return;
        }

        insert_recursive(node_p, point, id, xy + 1);
    }
};




