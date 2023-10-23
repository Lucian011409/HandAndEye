#include <opencv2\opencv.hpp>
#include<iostream>

using namespace cv;
using namespace std;

int main()
{

    //initialize and allocate memory to load the video stream from camera

    VideoCapture camera0(1);
    camera0.set(CAP_PROP_FRAME_WIDTH, 1920);
    camera0.set(CAP_PROP_FRAME_HEIGHT, 1080);

    if (!camera0.isOpened())
        return 1;

    int CountS = 0;

    string file_folder = ".\\Data\\";
    string type = ".bmp";

    namedWindow("img",0);
    resizeWindow("img", 960, 540);


    while (true) 
    {
        //grab and retrieve each frames of the video sequentially
        Mat3b frame0;
        camera0 >> frame0;


        if (frame0.empty() )
        {
            break;
        }
       
        imshow("img", frame0 );

        //wait for 30 milliseconds
        int c = waitKey(30);

        if (c>=0&&c!=27) //按除esc之外的任意键截取图像
        {
            if (0 == CountS)
            {
                string deleteImgPathL = "del " + file_folder  + "*" + type;
                system(deleteImgPathL.c_str());        
            }

            CountS++;
            stringstream ss;
            ss << setw(5) << setfill('0') << CountS;
            string imgPathL = file_folder  + ss.str() + type;
            imwrite(imgPathL, frame0);
   
        }
		else  if (27 == c)//exit the loop if user press "Esc" key  (ASCII value of "Esc" is 27)
            break;
    }

    return 0;
}