#include <iostream>
#include <stdio.h>
#include <unordered_map>
#include <assert.h>
#include <queue>
#include <tuple>
#include <cmath>

#include <opencv2/opencv.hpp>

//include custom headers and files
#include <utils.h>
#include <dijkstra.h>



namespace dji{
    

    //Coordinate constructor
    Coordinate::Coordinate(int r_c, int c_c) : r(r_c), c(c_c) {};


    //backtrack from last node to the start node
    int backtrack(Node* end, cv::Mat image, cv::Vec3b color){
        
        int length = 0;
        int r, c; // row and column
        while(end -> parent != nullptr){ // while we are not at the start node
            length++;
            image.at<cv::Vec3b>(end -> r, end -> c) = color;
            end = end -> parent; //set end to its parent
        }
        cv::circle(image, cv::Point(end -> c, end -> r), 5, color, 2);

        return length;
    }

    //run the planner
    Result planner(cv::Mat map, tuple<int, int> start, tuple<int, int> goal){
        // return map;
        // initialize traversable coordinates
        Coordinate deltas[8] {{-1,-1}, {-1,0}, {-1,1}, {0,-1}, {0,1}, {1,-1}, {1,0}, {1,1}};

        //define colors
        utils::Color colors;
        colors.assign_colors();

        //map information
        int cols = map.cols; //number of columns in the map
        int rows = map.rows; //number of rows in the map

        /*append to list of elements which all are no obstacle elements (white)*/
        int element = 0; //element number 

        unordered_map<int, Node*> node_map; //hashmap of nodes

        //check if start and goal are valid
        if (map.at<cv::Vec3b>(get<0>(start),get<1>(start)) == colors.c_black){
            cout << "not a valid start node: make sure you are on a white pixel" << endl;
        }

        if (map.at<cv::Vec3b>(get<0>(goal),get<1>(goal)) == colors.c_black){
            cout << "not a valid goal node: make sure you are on a white pixel" << endl;
        }

        /*create hashmap using linear indexing the generate the keys, 
        this allows for fast lookup of nodes based on their row and column*/
        Node* init = new Node(); //initialize the starting node
        for (int r=0; r<rows; r++){ //iterate over rows
            for (int c=0; c<cols; c++){ //iterate over columns

                cv::Vec3b image_color = map.at<cv::Vec3b>(r,c); //image color
                if (image_color == colors.c_white){ //if that row is free space, need to create a new node and append it
                    // map.at<cv::Vec3b>(r,c) = colors.c_blue;
                    Node* new_node = new Node(); //declare new node
                    new_node -> r = r;
                    new_node -> c = c;
                    new_node -> e = element;
                    node_map[element] = new_node; //add new node the node hashap

                    if ((new_node -> r == get<0>(start)) && (new_node -> c == get<1>(start))){
                        init = new_node;
                        init -> dist = 0;
                    }
                }

                //increment element
                element++;
            }
        }

        // sanity check
        for (int r=0; r<rows; r++){ //iterate over rows
            for (int c=0; c<cols; c++){ //iterate over columns

                element = r*cols + c;

                if (!(node_map.find(element) == node_map.end())){ //if key is found in the hashmap
                    Node* new_node = node_map[element];
                    assert(r == new_node -> r);
                    assert(c == new_node -> c);

                    // map.at<cv::Vec3b>(r,c) = c_orange;
                }
            }
        }
        // display_img(map);

        // iPair ==>  Integer Pair
        typedef pair<double, int> iPair; //distance and element index
        priority_queue<iPair, vector <iPair> , greater<iPair> > pq;

        init -> dist = 0; //start node has zero dist
        pq.push(make_pair(init -> c, init -> e)); //add start node to min priority queue, pushes to the end of the queue

        double current_dist;

        Node* current_node = new Node(); //declare current_node
        Node* neigh_node = new Node(); //the neighbor node

        int neigh_r, neigh_c; //neighbor row and col
        int neigh_e; //neighbor element
        double neigh_dist; //neighbor distance
        double trav_dist; //distance to travel to new node

        int iter = 0;

        while(!pq.empty()){ //while the unexplored set is not empty
            // cout << iter << endl;
            iter ++;

            iPair current_pair = pq.top(); //get the pair with the smallest distance
            current_dist = current_pair.first; //dist of current node
            element = current_pair.second; //element index of current node

            pq.pop(); //remove current node from priority queue

            current_node = node_map[element]; //get the current node from the hashmap
            current_node -> vis = 1; //current node has been visited

            map.at<cv::Vec3b>(current_node -> r,current_node -> c) = colors.c_pblue; //color visited node on map

            //check if current node is the goal, return
            if ((current_node -> r == get<0>(goal)) && (current_node -> c == get<1>(goal))){
                cout << "goal has been found!" << endl;
                break;
            }

            //for each neighbor of current node still in the unexplored set
            for (Coordinate coord:deltas){//iterate through all the coordinates to get the neighbors

                neigh_r = current_node -> r + coord.r;//row of neighbor
                neigh_c = current_node -> c + coord.c;//col of neighbor
                neigh_e = neigh_r*cols + neigh_c;
                
                //check to see that coordinates are in bound
                if ((neigh_c < 0 || neigh_c >= cols) || (neigh_r<0 || neigh_r>=rows) ){
                    // cout << "out of bounds" << endl;
                    continue; //continue
                }

                //check to see if neighbor is in the unexplored set
                if (node_map.find(neigh_e) == node_map.end()){ //if the neighbor is not found
                    // cout << "neighbor not in unexplored set" << endl;
                    continue; //continue
                }

                /*calculate the new distance:
                the new distance is the current nodes distance plus the distance between current
                node and the neighbor*/
                neigh_node = node_map[neigh_e]; //get the neighbor node

                if (neigh_node -> vis==1){ //if current node has already been visited         
                    //cout << "neighbor has already been visited" << endl;       
                    continue;
                }

                trav_dist = 1; //traversal distance
                if ((abs(coord.c) + abs(coord.r)) > 1){//slightly higher traversal distance for diagonal
                    trav_dist = 1.1;
                }

                neigh_dist = current_dist + trav_dist; //calculate the new proposed distance

                /*if the new distance is less than the neighbor nodes distance:
                set neighbors distance to the new distance
                set neighbors parent to the current node.
                Then push that neighbor to the priority queue*/
                if (neigh_dist < neigh_node -> dist){

                    neigh_node -> dist = neigh_dist; //update dist of the node
                    neigh_node -> parent = current_node; //assign current node as parent of the neighbor node
                    node_map[neigh_e] = neigh_node; //update neighbor node in the hash map
                    pq.push(make_pair(neigh_dist, neigh_node -> e)); //push neighbor distance and element to the priority queue

                    //update neigh parent
                    map.at<cv::Vec3b>(neigh_r,neigh_c) = colors.c_red; //color neighbor node added to queue
                }
            }

            if (pq.empty()){
                cout << "failed to find goal :(" << endl;
            }

        }

        
        //backtrack to get the shortest path
        int length = backtrack(current_node, map, colors.c_green);

        cv::circle(map, cv::Point(current_node -> c, current_node -> r), 5, colors.c_green, 2); //fill in end node

        //add results to Result structure to return
        Result result;
        result.length = length;
        result.map = map;

        return result;
    }

}