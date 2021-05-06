#include <iostream>
#include <vector>
#include <stdio.h>
#include <unordered_map>
#include <assert.h>
#include <queue>
#include <tuple>
// #include <map>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>


using namespace std; //set namespace 

//class for nodes
class Node {

    public:
    int r, c; // row and column of the node
    int e; // the element number
    int vis = 0; //has been visited or not
    double dist = numeric_limits<double>::infinity();


    Node* parent; //define the parent, parent is a pointer to a Node

    //Node(int row, int col, int ele) : r(row), c(col), e(ele) {}; //constructor

};

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

int main(int argc, char** argv) {

    //traversable coordinates
    Coordinate deltas[8] {{-1,-1}, {-1,0}, {-1,1}, {0,-1}, {0,1}, {1,-1}, {1,0}, {1,1}};
    //start and goal locations
    auto start = make_tuple(310, 50); //(row, column)
    auto goal = make_tuple(130, 520);

    //colors
    cv::Vec3b c_blue;
    c_blue[0] = 255;
    c_blue[1] = 0;
    c_blue[2] = 0;

    cv::Vec3b c_green;
    c_blue[0] = 0;
    c_blue[1] = 255;
    c_blue[2] = 0;

    cv::Vec3b c_white; //free space
    c_white[0] = 255;
    c_white[1] = 255;
    c_white[2] = 255;

    cv::Vec3b c_black; //obstacle space
    c_black[0] = 0;
    c_black[1] = 0;
    c_black[2] = 0;


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

    unordered_map<int, Node> node_map; //hashmap of nodes

    Node init;

    for (int r=0; r<rows; r++){ //iterate over rows
        for (int c=0; c<cols; c++){ //iterate over columns

            cv::Vec3b image_color = map.at<cv::Vec3b>(r,c); //image color
            if (image_color == c_white){ //if that row is free space, need to create a new node and append it
                // map.at<cv::Vec3b>(r,c) = c_blue;
                Node new_node; //declare new node
                new_node.r = r;
                new_node.c = c;
                new_node.e = element;
                node_map[element] = new_node; //add new node the node hashap

                if ((new_node.r == get<0>(start)) && (new_node.c == get<1>(start))){
                    init = new_node;
                    init.dist = 0;
                }


            }

            //increment element
            element++;
        }
    }

    //sanity check
    for (int r=0; r<rows; r++){ //iterate over rows
        for (int c=0; c<cols; c++){ //iterate over columns

            element = r*cols + c;

            if (!(node_map.find(element) == node_map.end())){ //if key is found in the hashmap
                Node new_node = node_map[element];
                // cout << "actual: " << r << " " << c << endl;
                // cout << "found: " << new_node.r << " " << new_node.c << endl;
                assert(r == new_node.r);
                assert(c == new_node.c);

            }
        }
    }

    // cout << nodes.size() << endl;
    // display_img(map);

    // cout << node_map.size();

    // iPair ==>  Integer Pair
    typedef pair<double, int> iPair; //distance and element index
    priority_queue<iPair, vector <iPair> , greater<iPair> > pq;

    // cout << init.r << init.c << endl;
    init.dist = 0; //start node has zero dist
    pq.push(make_pair(init.c, init.e)); //add start node to min priority queue, pushes to the end of the queue
    // node_map.erase(init.e); //remove start node from the hash map

    double current_dist;

    Node current_node; //declare current_node
    Node neigh_node; //the neighbor node

    int neigh_r, neigh_c; //neighbor row and col
    int neigh_e; //neighbor element
    int neigh_dist; //neighbor distance

    int iter = 0;

    while(!pq.empty()){ //while the unexplored set is not empty
        // cout << iter << endl;
        iter ++;
        // cout << "in loop" << endl;
        // element = pq.pop().second;
        iPair current_pair = pq.top(); //get the pair with the smallest distance
        current_dist = current_pair.first; //dist of current node
        element = current_pair.second; //element index of current node

        // cout << "current dist " << current_dist << endl;
        // cout << "current element " << element << endl;

        cout << pq.size() << endl;
        pq.pop(); //remove current node from priority queue
        // cout << "pq_size " << pq.size() << endl;
        cout << pq.size() << endl;

        if (node_map.find(element) == node_map.end()){ //if the neighbor is not found
                cout << "---------------------" << endl;
        }

        current_node = node_map[element]; //get the current node from the hashmap
        cout << "visited" << current_node.vis << endl;
        current_node.vis = 1; //current node has been visited

        map.at<cv::Vec3b>(current_node.r,current_node.c) = c_green;

        cout << current_node.dist << endl;
        // node_map.erase(element); //remove current node from the hashmap
        // cout << "node_map size" << node_map.size() << endl;

        // cout << "element " << element << endl;
        // cout << "current_node coords " << current_node.r << current_node.c << endl;


        //check if current node is the goal, return
        if ((current_node.r == get<0>(goal)) && (current_node.c == get<1>(goal))){
            cout << "goal has been found!" << endl;
            break;
        }

        //for each neighbor of current node still in the unexplored set
        for (Coordinate coord:deltas){//iterate through all the coordinates to get the neighbors
            // cout << "check coord" << endl;
            neigh_r = current_node.r + coord.r;//row of neighbor
            neigh_c = current_node.c + coord.c;//col of neighbor
            neigh_e = neigh_r*cols + neigh_c;

            // cout << "delta" << coord.r << coord.c << endl;
            // cout << "current" << current_node.r << current_node.c << endl;
            // cout << neigh_r << neigh_c << endl;
            
            //check to see that coordinates are in bound
            if ((neigh_c < 0 || neigh_c >= cols) || (neigh_r<0 || neigh_r>=rows) ){
                // cout << "out of bounds" << endl;
                continue; //continue
            }

            // cout << "neigh_e " << neigh_e << endl;
            //check to see if neighbor is in the unexplored set
            if (node_map.find(neigh_e) == node_map.end()){ //if the neighbor is not found
                // cout << "neighbor not in unexplored set" << endl;
                // cout << "neigh_e inside " << neigh_e << endl;
                continue; //continue
            }

            /*calculate the new distance:
            the new distance is the current nodes distance plus the distance between current
            node and the neighbor*/
            neigh_node = node_map[neigh_e]; //get the neighbor node

            if (neigh_node.vis==1){ //if current node has already been visited

                continue;
            }

            neigh_dist = current_dist + 1; //calculate the new proposed distance

            // cout << "check" << endl;
            /*if the new distance is less than the neighbor nodes distance:
            set neighbors distance to the new distance
            set neighbors parent to the current node.
            Then push that neighbor to the priority queue*/
            // cout << "neigh_dist" << neigh_dist << endl;
            if (neigh_dist < neigh_node.dist){

                // cout << "neigh_dist " << neigh_dist << endl;
                cout << "neigh_node.dist " << neigh_node.dist << endl;
                neigh_node.dist = neigh_dist; //update dist of the node
                neigh_node.parent = &current_node;
                node_map[neigh_e] = neigh_node;
                pq.push(make_pair(neigh_dist, neigh_node.e));

                // cout << "push" << endl;
                // cout << neigh_node.r << endl;
                // cout << neigh_node.c << endl;
                // cout << "neigh_e " << neigh_e << endl;
                cout << "pushed neigh_node.e " << neigh_node.e << endl;

                //update neigh parent
                map.at<cv::Vec3b>(neigh_r,neigh_c) = c_blue;
            }
        }
        cv::imshow("Display window", map); //display image
        cv::waitKey(0);
        if(iter == 250000){
        break;}

        if (pq.empty()){
            cout << "failed to find goal :(" << endl;
        }

    }

    cv::circle(map, cv::Point(current_node.c, current_node.r), 5, cv::Scalar(0, 255, 0), 2);
    display_img(map);
}

