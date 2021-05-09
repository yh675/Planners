#include <iostream>
#include <vector>
#include <stdio.h>
#include <unordered_map>
#include <assert.h>
#include <queue>
#include <tuple>
// #include <map>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

//include custom headers and files
#include <utils.h>


using namespace std; //set namespace 

//class for nodes
class Node {

    public:
        int r, c; // row and column of the node
        int e; // the element number
        int vis = 0; //has been visited or not
        double dist = numeric_limits<double>::infinity();

    Node* parent = nullptr; //define the parent, parent is a pointer to a Node

    //Node(int row, int col, int ele) : r(row), c(col), e(ele) {}; //constructor

};

// class Color{

//     public:
//         //colors
//         cv::Vec3b c_blue;

//     void assign_colors(){
//         c_blue[0] = 255;
//         c_blue[1] = 0;
//         c_blue[2] = 0;
//     }

// };

//class for coordinates of traversal
class Coordinate {

    public:
        int r, c;

    Coordinate(int r_c, int c_c) : r(r_c), c(c_c) {};
};


//binary thresholding
cv::Mat bin_thresh(cv::Mat image){
    
    cv::Mat dst; //destination 
    int thresh = 127; //thresholding
	int maxValue = 255;
    cv::threshold(image, dst, thresh, maxValue, CV_THRESH_BINARY);

    return dst;
}

//display image with waitKey on 'ESC' built in
void display_img(cv::Mat image){
    cv::imshow("Display window", image); //display image
    while(cv::waitKey(1) != 27); //wait until esc is pressed before closing image
}

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


int main(int argc, char** argv) {

    //traversable coordinates
    Coordinate deltas[8] {{-1,-1}, {-1,0}, {-1,1}, {0,-1}, {0,1}, {1,-1}, {1,0}, {1,1}};
    //start and goal locations
    auto start = make_tuple(310, 50); //(row, column)
    auto goal = make_tuple(130, 520);

    //define colors
    Color colors;
    colors.assign_colors();

    string map_dir = "Maps/World_Map_bin.png"; //directory to load image map
    cv::Mat map; //initialize image object
    map = cv::imread(map_dir, CV_LOAD_IMAGE_COLOR); //load map 

    // cv::circle(map, cv::Point(get<1>(start), get<0>(start)), 5, cv::Scalar(0, 0, 255), 2);
    // cv::circle(map, cv::Point(get<1>(goal), get<0>(goal)), 5, cv::Scalar(0, 255, 0), 2);
    // display_img(map);

    int cols = map.cols; //number of columns in the map
    int rows = map.rows; //number of rows in the map
    //int ch = map.channels(); //number of channels

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
        // cv::imshow("Display window", map); //display image
        // cv::waitKey(0);

        // if (iter == 50000){
        //     cv::imshow("Display window", map); //display image
        //     cv::waitKey(0);
        //     display_img(map);
        // }

        if (pq.empty()){
            cout << "failed to find goal :(" << endl;
        }

    }

    
    //backtrack to get the shortest path
    int length = backtrack(current_node, map, colors.c_green);
    cout << "length: " << length << endl;

    cv::circle(map, cv::Point(current_node -> c, current_node -> r), 5, colors.c_green, 2);
    display_img(map);
}

