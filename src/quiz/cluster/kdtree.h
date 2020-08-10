/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <queue>


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

        std::queue<Node*> queue {};
        queue.push(root);
        int xy = 0;
		while (!queue.empty()) {
            auto node = queue.front();

            if (in_tolerance_box(target, distanceTol, node->point)) {
                ids.push_back(node->id);
            }

            // check if target +/- distance is bigger of smaller than the splitting point
            if (target[xy % 2] + distanceTol >= node->point[xy % 2]) {
                if (node->right != nullptr) {
                    queue.push(node->right);
                }
            }
            if (target[xy % 2] - distanceTol <= node->point[xy % 2]) {
                if (node->left != nullptr) {
                    queue.push(node->left);
                }
            }

            xy += 1;
            queue.pop();
		}

		return ids;
	}

private:
    bool in_tolerance_box(std::vector<float> box, float distanceTol, std::vector<float> point)
    {
        if (
            in_tolerance(box[0], distanceTol, point[0]) &&
            in_tolerance(box[1], distanceTol, point[1])) {
            return true;
        }

        return false;
    }

    static bool in_tolerance(float value_center, float distanceTol, float value)
    {
        auto value_add = value_center + distanceTol;
        auto value_sub = value_center - distanceTol;

	    if (value_sub <= value && value_add >= value) {
	        return true;
	    }

	    return false;
    }

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




