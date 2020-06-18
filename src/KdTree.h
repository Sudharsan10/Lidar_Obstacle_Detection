
#ifndef PLAYBACK_KDTREE_H
#define PLAYBACK_KDTREE_H

#include <iostream>
#include <vector>

//template<typename PointT>
//struct Node {
//    PointT point;
//    int id;
//    Node<PointT>* left;
//    Node<PointT>* right;
//
//    Node<PointT>(PointT arr, int setId)
//            :	point(arr), id(setId), left(NULL), right(NULL)
//    {}
//};
//
//
//template<typename PointT>
//struct KdTree {
//    Node<PointT>* root;
//
//    KdTree<PointT>(): root{NULL}{}
//
//    void insert(PointT point, int id) {
//        insertHelper3D(&root, 0, point, id);
//    }
//
//    void insertHelper3D(Node<PointT>** node, uint depth, PointT point, int id) {
//        uint splitId = depth%3;
//        if (*node == NULL)
//            *node = new Node<PointT>(point, id);
//        else if(splitId == 0){
//            if (point.x < (*node)->point.x)
//                insertHelper3D(&(*node)->left, depth+1, point, id);
//            else
//                insertHelper3D(&(*node)->right, depth+1, point, id);
//        }
//        else if(splitId == 1){
//            if(point.y < (*node)->point.y)
//                insertHelper3D(&(*node)->left, depth+1, point, id);
//            else
//                insertHelper3D(&(*node)->right, depth+1, point, id);
//        }
//        else if(splitId == 2){
//            if(point.z < (*node)->point.z)
//                insertHelper3D(&(*node)->left, depth+1, point, id);
//            else
//                insertHelper3D(&(*node)->right, depth+1, point, id);
//        }
//    }
//
//    std::vector<int> search(PointT target, float distanceTol){
//        std::vector<int> ids;
//        searchHelper3D(target, root, 0, distanceTol, ids);
//        return ids;
//    }
//
//    void searchHelper3D(PointT target, Node<PointT>* node, uint depth, float distanceTol, std::vector<int>& ids) {
//        if (node != NULL) {
//            if ((node->point.x >= (target.x - distanceTol) && node->point.x <= (target.x + distanceTol))
//                && (node->point.y >= (target.y - distanceTol) && node->point.y <= (target.y + distanceTol))
//                && (node->point.z >= (target.z - distanceTol) && node->point.z <= (target.z + distanceTol))) {
//                float distance = sqrt((node->point.x - target.x) * (node->point.x - target.x) +
//                                      (node->point.y - target.y) * (node->point.y - target.y) +
//                                      (node->point.z - target.z) * (node->point.z - target.z));
//                if (distance <= distanceTol)
//                    ids.push_back(node->id);
//            }
//
//
//            uint splitId = depth % 3;
//            if (splitId == 0)
//                if ((target.x - distanceTol) < node->point.x)
//                    searchHelper3D(target, node->left, depth + 1, distanceTol, ids);
//                else
//                    searchHelper3D(target, node->right, depth + 1, distanceTol, ids);
//            else if (splitId == 1)
//                if ((target.y - distanceTol) < node->point.y)
//                    searchHelper3D(target, node->left, depth + 1, distanceTol, ids);
//                else
//                    searchHelper3D(target, node->right, depth + 1, distanceTol, ids);
//            else if (splitId == 2) {
//                if ((target.z - distanceTol) < node->point.z)
//                    searchHelper3D(target, node->left, depth + 1, distanceTol, ids);
//                else
//                    searchHelper3D(target, node->right, depth + 1, distanceTol, ids);
//            }
//        }
//    }
//};

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
            : root{NULL}
    {}

    void insert(std::vector<float> point, int id)
    {

        // TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root
        insertHelper3D(&root, 0, point, id);

    }

    void insertHelper3D(Node** node, uint depth, std::vector<float> point, int id)
    {
        if (*node == NULL)
            *node = new Node(point, id);
        else
        {
            uint dfac = depth%3;
            if(point[dfac] < ((*node)->point[dfac]))
                insertHelper3D(&(*node)->left, depth+1, point, id);
            else
                insertHelper3D(&(*node)->right, depth+1, point, id);
        }

    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol)
    {
        std::vector<int> ids;
        searchHelper3D(target, root, 0, distanceTol, ids);
        return ids;
    }

    void searchHelper3D(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int> &ids){
        if(node!=NULL)
        {
            if( (node->point[0] >= (target[0] - distanceTol) && node->point[0]<=(target[0]+distanceTol))
                &&	(node->point[1] >= (target[1] - distanceTol) && node->point[1]<=(target[1]+distanceTol))
                &&	(node->point[2] >= (target[2] - distanceTol) && node->point[2]<=(target[2]+distanceTol)) )
            {
                float distance = sqrt( 	(node->point[0] - target[0])*(node->point[0] - target[0])
                                          + (node->point[1] - target[1])*(node->point[1] - target[1])
                                          + (node->point[2] - target[2])*(node->point[2] - target[2]) );
                if (distance <= distanceTol)
                    ids.push_back(node->id);
            }

            if ((target[depth%3]-distanceTol)<node->point[depth%3])
                searchHelper3D(target, node->left, depth+1, distanceTol, ids);
            if ((target[depth%3]+distanceTol)>node->point[depth%3])
                searchHelper3D(target, node->right, depth+1, distanceTol, ids);
        }
    }

};
#endif //PLAYBACK_KDTREE_H
