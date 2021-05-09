#ifndef UTILS
#define UTILS

#include <stdio.h>
#include <opencv2/opencv.hpp>

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

    void assign_colors(){
        c_blue[0] = 255;
        c_blue[1] = 0;
        c_blue[2] = 0;

        c_pblue[0] = 255;
        c_pblue[1] = 191;
        c_pblue[2] = 0;

        c_indigo[0] = 130;
        c_indigo[1] = 0;
        c_indigo[2] = 75;

        c_yellow[0] = 0;
        c_yellow[1] = 255;
        c_yellow[2] = 255;

        c_orange[0] = 0;
        c_orange[1] = 128;
        c_orange[2] = 255;

        c_green[0] = 0;
        c_green[1] = 255;
        c_green[2] = 0;

        c_red[0] = 0;
        c_red[1] = 0;
        c_red[2] = 255;

        c_white[0] = 255;
        c_white[1] = 255;
        c_white[2] = 255;

    }

};

int add(int x, int y); // function prototype for add.h -- don't forget the semicolon!

#endif