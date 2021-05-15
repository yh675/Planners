#include <iostream>
#include <stdio.h>
#include <tuple>
#include <string>

#include <opencv2/opencv.hpp>
// #include <Eigen/Core>

//include custom headers and files
#include <utils.h>
#include <dijkstra.h>

using namespace std; //set namespace 


int main(int argc, char* argv[]) {

    //load the map
    string map_dir = "Maps/World_Map_bin.png"; //directory to load image map
    cv::Mat map; //initialize image object
    map = cv::imread(map_dir, CV_LOAD_IMAGE_COLOR); //load map 

    //start and goal locations
    // auto start = make_tuple(310, 50); //(row, column)
    // auto goal = make_tuple(130, 520);
    tuple <int, int> start, goal;
    start = make_tuple(310, 50); //(row, column)
    goal = make_tuple(130, 520);


    string planner_type = argv[1]; //which planner type it is

    dji::Result result; //define Result structure to store length and filled in map

    if (planner_type == "dij"){ //run Dijkstras      
        cout << "Running Dijkstras Algorithm" << endl;
        result = dji::planner(map, start, goal, planner_type); //run planner
    }

    else if (planner_type == "astar"){ //run A*
        cout << "Running A* Algorithm" << endl;
        result = dji::planner(map, start, goal, planner_type); //run planner
    }

    else { // ask for a valid prompt
        cout << "Please enter a valid planner type" << endl;
        cout << "Valid Planner types are: dij, astar" << endl;
        return 0;
    }

    //display results
    cout << "Path Length: " << result.length << endl; //print path length
    utils::display_img(result.map); //visualize result
    
    //cv::imwrite("Maps/Astar_result.png", map); //write the results

    return 0;
 
}

