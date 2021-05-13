#ifndef UTILS_H
#define UTILS_H

#include <stdio.h>
#include <opencv2/opencv.hpp>

namespace utils{ //define namespace

    //Class which holds colors
    class Color{
        public: //declare vectors
            //colors
            cv::Vec3b c_blue;
            cv::Vec3b c_pblue;
            cv::Vec3b c_indigo;
            cv::Vec3b c_yellow;
            cv::Vec3b c_orange;
            cv::Vec3b c_green;
            cv::Vec3b c_red;
            cv::Vec3b c_white; //free space
            cv::Vec3b c_black; //obstacle space
        
        void assign_colors(); //assign colors
    };

    //binary thresholding
    cv::Mat bin_thresh(cv::Mat image);

    //display image with waitKey on 'ESC' built in
    void display_img(cv::Mat image);

}

#endif