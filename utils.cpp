#include <stdio.h>
#include <math.h>
#include <utils.h>
#include <opencv2/opencv.hpp>

using namespace std;

namespace utils{

    //assign colors
    void Color::assign_colors(){
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

}