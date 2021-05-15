#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <stdio.h>
#include <limits>
#include <iostream>
#include <tuple>


using namespace std;

namespace dji{

    //result
    struct Result
    {
        int length;
        cv::Mat map;
    };


    //class for nodes
    class Node {

        public:
            int r, c; // row and column of the node
            int e; // the element number
            int vis = 0; //has been visited or not
            double dist = numeric_limits<double>::infinity(); //current distance, equivalent of g in astar
            double cost = 0.; //cost for astar planner, equivalent of f in astar, in f = g + h, where h is the heuristic

        Node* parent = nullptr; //define the parent, parent is a pointer to a Node

        //Node(int row, int col, int ele) : r(row), c(col), e(ele) {}; //constructor

    };

    //declare Coordinate for traversal deltas
    class Coordinate {

        public:
            int r, c;

        // Coordinate(int r_c, int c_c) : r(r_c), c(c_c) {};
        Coordinate(int r_c, int c_c);
    };

    //backtrack from last node to the start node
    int backtrack(dji::Node* end, cv::Mat image, cv::Vec3b color);

    // run the planner
    Result planner(cv::Mat map, tuple<int, int> start, tuple<int, int> goal, string planner_type);

}

#endif