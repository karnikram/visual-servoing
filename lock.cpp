
Conversation opened. 1 read message.

Skip to content
Using Gmail with screen readers
 Streak
Karnik
Search


aadithyavenkat@gmail.com 


Take me to Inbox
Gmail
COMPOSE
Labels
Inbox (6)
Starred
Important
Sent Mail
Recently Viewed
All Tracked Emails
Awaiting Reply
Drafts (2)
Snoozed
Pipelines + New
Career Search
Circles
Notes
Personal
Travel
More labels 
Hangouts

 
 
  
Move to Inbox 
  
More 
12 of about 59  
 
Print all In new window
Kurukshetra OpenCV Code

Karnik Ram <karnikram@gmail.com>
AttachmentsFeb 22

to aadithya 
Find attached.
Attachments area
	
Click here to Reply or Forward
7.77 GB (51%) of 15 GB used
Manage
Terms - Privacy
Last account activity: 0 minutes ago
Details
aadithya venkat
Add to circles

Show details


#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <stdlib.h>
#include<SerialStream.h>
#include <sstream>
#include <string.h>

using namespace cv;
using namespace std;
using namespace LibSerial;

#define PI 3.14159265358979323846
#define CODE_LENGTH 4
int code[] = {1,0,1,0};

VideoCapture video_input(1);
Mat src, src_HSV, thresholded, templ_one, templ_zero, result, track;
double minVal; double maxVal; Point minLoc; Point maxLoc;
int result_cols, result_rows;
Point matchLoc;
Point cone, czero;

Mat b1, b2;

vector<vector<Point> > contours1, contours2;
vector<Vec4i> hierarchy1, hierarchy2;
int max_area, best_cnt1, best_cnt2;
Point cb1(0,0), cb2(0,0);
Moments mb1,mb2;

int nLowH = 124;
int nHighH = 179;
int nLowS = 0; 
int nHighS = 255;
int nLowV = 59;
int nHighV = 255;

// pink

int b1LowH = 150;
int b1HighH = 179;
int b1LowS = 0;
int b1HighS = 255;
int b1LowV = 160;
int b1HighV = 255;

//yellow

int b2LowH = 0;
int b2HighH = 50;
int b2LowS = 70;
int b2HighS = 255;
int b2LowV = 131;
int b2HighV = 251;

int c, i, code_index = 0;

double dot, det, dheading;

SerialStream arduino;

bool gate = 1;

char* image_window = "Found Match";

void matching_method();

double goal_heading(Point);

void go_to_goal(Point);

void get_xy();

class Bot
{
    public:
        double wradius,blength,bvelocity,pwm_lower,pwm_upper,omega_lower,omega_upper,rpm_lower,rpm_upper;
        int wvelocities[2];
        double Kp,Ki,Kd,error,prev_error,integ;
        long int time,prev_time,freq;

        Bot(double p)
        {
            Kp=p;
            error=0;prev_error=0;integ=0;
            time=0;prev_time=0;
            freq = getTickFrequency();
            wradius = 0.035;
            blength = 0.10;
            bvelocity = 0.065;
            pwm_lower = 80.0;
            pwm_upper = 240.0;
            omega_lower = 0.0;
            omega_upper = 8.35;

        }

        void Uni2DiffKinematics(double w)
        {
            double sum  = 2*bvelocity/(wradius);
            double diff = blength*w/(wradius);
            double l_w = (sum+diff)/2;
            double r_w = (sum-diff)/2;
            if ((l_w > omega_upper) || (r_w > omega_upper))
            {
                if (l_w > omega_upper)
                {
                    double temp = l_w - omega_upper;
                    l_w = omega_upper;
                    r_w -= temp;
                    if(r_w < omega_lower)
                    {
                        r_w =omega_lower;
                    }

                }
                else
                {
                    double temp = r_w - omega_upper;
                    r_w = omega_upper;
                    l_w -= temp;
                    if(l_w < omega_lower)
                    {
                        l_w=omega_lower;
                    }
                }
            }
            else if ((l_w < omega_lower) || (r_w < omega_lower))
            {
                if(l_w < omega_lower)
                {
                    double temp = omega_lower - l_w;
                    l_w = omega_lower;
                    r_w += temp;
                    if (r_w > omega_upper)
                    {r_w = omega_upper;}
                }
                else
                {
                    double temp = omega_lower - r_w;
                    r_w = omega_lower;
                    l_w += temp;
                    if(l_w > omega_upper)
                    {
                        l_w = omega_upper;
                    }
                }
            }

            l_w = pwm_lower + (((pwm_upper - pwm_lower)/(omega_upper - omega_lower))*l_w);
            r_w = pwm_lower + (((pwm_upper - pwm_lower)/(omega_upper - omega_lower))*r_w);

            wvelocities[1]=int(l_w);
            wvelocities[0]=int(r_w);

        }

        void ResetController()
        {
            prev_time = getTickCount();
            error=0;prev_error=0;integ=0;
        }

        double PID_Controller(double accel)
        {
            double signal;
            error = accel;
            integ+=error;
            time = getTickCount();
            double dt = (double(time - prev_time))/freq;
            signal = (error*Kp) + ((error - prev_error)*Kd*dt) + (integ*Ki/dt);
            prev_error = error;
            prev_time = time;
            return signal;
        }
} driver(3);
       
int main(int argc, char ** argv)
{
	if(!video_input.isOpened())
	{
		std::cout<<"No video input"<<std::endl;
		return -1;
	}
	
	arduino.Open("/dev/ttyUSB0");
	arduino.SetBaudRate(SerialStreamBuf :: BAUD_9600);
	arduino.SetCharSize(SerialStreamBuf :: CHAR_SIZE_8);

	video_input.read(src);	
	cvtColor(src, src_HSV, COLOR_BGR2HSV);

	templ_one = imread("one.jpg",0);
	templ_zero = imread("zero.jpg",0);

	namedWindow(image_window, CV_WINDOW_AUTOSIZE);

	while(true)
	{	
		
		get_xy();
		imshow(image_window, src);
		
		while(code_index < CODE_LENGTH)
		{
			driver.ResetController();	
				if(code[code_index] == 0)
				{
				while(gate)
				{
					go_to_goal(czero);
					get_xy();
					imshow(image_window, src);
					waitKey(20);
					gate = (double(norm(cb1-czero)>20)); 
					if(gate == 0)
					{
							arduino << "red!";
							usleep(5000000);
					}
				}
				
				gate = 1;
				}

				else if(code[code_index] == 1)
				{	
					while(gate)
					{
					go_to_goal(cone);
					get_xy();
					imshow(image_window, src);
					waitKey(20);
					gate = (double(norm(cb1-cone)>20)); 
					if(gate == 0)
					{
							arduino << "green!";
							usleep(5000000);
					}
				}
					gate = 1;
				}

			code_index++;
	}
				 
	c = waitKey(20);

	if((char)c == 27)
	{
		break;
  	}	
}

	arduino.Close();
	return 0;
}

void matching_method()
{

//	One

	result_cols = thresholded.cols - templ_one.cols + 1;
	result_rows = thresholded.rows - templ_one.rows + 1;
	
	
	result.create(result_rows, result_cols, CV_32FC1);
	
	
	matchTemplate(thresholded, templ_one, result, CV_TM_CCOEFF_NORMED);
	normalize(result, result, 0, 1, NORM_MINMAX, -1);
	minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);
	
	matchLoc = maxLoc;

//	rectangle(src, matchLoc, Point(matchLoc.x + templ_one.cols , matchLoc.y + templ_one.rows), Scalar::all(0), 2, 8, 0);

	cone.x = (matchLoc.x + matchLoc.x + templ_one.cols) / 2;
	cone.y = (matchLoc.y + matchLoc.y + templ_one.rows) / 2;

	circle(src, cone, 5, Scalar::all(0), 2, 8, 0);
	

//	Zero

	result_cols = thresholded.cols - templ_zero.cols + 1;
	result_rows = thresholded.rows - templ_zero.rows + 1;
	
	result.create(result_rows, result_cols, CV_32FC1);

	matchTemplate(thresholded, templ_zero, result, CV_TM_CCOEFF_NORMED);
	normalize(result, result, 0, 1, NORM_MINMAX, -1);
	minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);

	matchLoc = maxLoc;

	//rectangle(src, matchLoc, Point(matchLoc.x + templ_zero.cols , matchLoc.y + templ_zero.rows), Scalar::all(0), 2, 8, 0);

	czero.x = (matchLoc.x + matchLoc.x + templ_zero.cols) / 2;
	czero.y = (matchLoc.y + matchLoc.y + templ_zero.rows) / 2;

	circle(src, czero, 5, Scalar::all(0), 2, 8, 0);
}

void get_xy()
{
	video_input.read(src);	
	cvtColor(src, src_HSV, COLOR_BGR2HSV);

	//Digits Match

	inRange(src_HSV,Scalar(nLowH,nLowS,nLowV),Scalar(nHighH,nHighS,nHighV),thresholded);

	erode(thresholded, thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
	dilate(thresholded, thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5))); 

	dilate(thresholded, thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5))); 
	erode(thresholded, thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
	
	matching_method();

	//B1, B2 Centroids

	inRange(src_HSV, Scalar(b1LowH, b1LowS, b1LowV), Scalar(b1HighH, b1HighS, b1HighV), b1);

	erode(b1, b1, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
	dilate(b1, b1, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
	
	dilate(b1, b1, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
	erode(b1, b1, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));


	inRange(src_HSV, Scalar(b2LowH, b2LowS, b2LowV), Scalar(b2HighH, b2HighS, b2HighV), b2);
	
	erode(b2, b2, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
	dilate(b2, b2, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
	
	dilate(b2, b2, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
	erode(b2, b2, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

	findContours(b1,contours1, hierarchy1, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
        findContours(b2,contours2, hierarchy2, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	
        for(i = 0; i < contours1.size(); i++)
        { 
            if (contourArea(contours1[i])> max_area)
            {
                max_area = contourArea(contours1[i]);
                best_cnt1 = i;
            }
        }

        for(i = 0; i < contours2.size(); i++)
        { 
            if (contourArea(contours2[i]) > max_area)
            {
                max_area = contourArea(contours2[i]);
                best_cnt2 = i;
            }
        }

        if (!((contours1.size()==0) || (contours2.size()==0)))
        {
            mb1 = moments(contours1[best_cnt1]);
            mb2 = moments(contours2[best_cnt2]);

            cb1= Point2f( mb1.m10/mb1.m00,mb1.m01/mb1.m00 );
            cb2 = Point2f( mb2.m10/mb2.m00,mb2.m01/mb2.m00 );

            circle(src, cb1, 3, Scalar(0,0,0), -1, 8, 0 );
            circle(src, cb2, 3, Scalar(0,0,0), -1, 8, 0 );
        }

}

double goal_heading(Point goal)        
{
	Point bot_dir = cb1 - cb2;
	Point goal_dir = cb1 - goal;

	dot = bot_dir.x*goal_dir.x + bot_dir.y*goal_dir.y;
	det = bot_dir.x*goal_dir.y - bot_dir.y*goal_dir.x;

	return (atan2(det,dot));
}

void go_to_goal(Point goal)
{	
	string s3,s4,s5;
	ostringstream s1,s2;
	dheading = goal_heading(goal);

	if(abs(dheading*180/PI)<15)
	{
		driver.bvelocity = 0.405;
	}
	
	else
	{

		driver.bvelocity = 0.025;
	}

	double signal = driver.PID_Controller(dheading); 
	driver.Uni2DiffKinematics(signal);

	s1 << driver.wvelocities[0];
	s2 << driver.wvelocities[1];

	s3 = s1.str();
	s4 = s2.str();

	if (s3.length()<3)
        s3 = "0"+s3;

	if (s4.length()<3)
	s4 = "0"+s4; 
                        
	
	s5 = "$"+ s3 + "#" + s4 + "!";

	const char * str = s5.c_str();

	arduino.write(str,9);

	cout<<str<<endl;
		
}
prelims.cppOpen
Displaying prelims.cpp.
