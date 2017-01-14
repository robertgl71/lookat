/*
 * lookat.cpp
 *
 *  Created on: 4 ene. 2017
 *      Author: robert
 */
//OpenCV
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

//std
#include<iostream>
#include<cmath>

#include "ros/ros.h"
#include "ros/package.h"
#include "geometry_msgs/Point.h"

using namespace cv;
using namespace std;

/** Function Headers */
Point detectFace( Mat frame );
int distEuclidean(int x1,int y1,int x2, int y2);

/** Global variables */
//String face_cascade_name = "haarcascade_frontalface_alt.xml";
CascadeClassifier face_cascade;

int main(int argc, char **argv)
{
	VideoCapture camera;
	Mat frame;
	int cam_id;
	Point prevPoint;

	ros::init(argc, argv, "point_publisher");
	ros::NodeHandle n;


	ros::Publisher point_pub = n.advertise<geometry_msgs::Point>("pointpub", 1000);
	ros::Rate loop_rate(10);

	std::string face_cascade_name_std;
	if (!n.getParam("face_cascade_name", face_cascade_name_std))
	      ROS_ERROR("Could not get face_cascade_name");
	cv::String face_cascade_name = face_cascade_name_std;



	//Check args
	switch(argc)
	{
		case 1: //no argument provided, so try /dev/video0
			cam_id = 0;
			break;
		case 2: //an argument is provided. Get it and set cam_id
			cam_id = atoi(argv[1]);
			break;
		default:
			std::cout << "Invalid number of arguments. Call program as: webcam_capture [video_device_id]. " << std::endl;
			std::cout << "EXIT program." << std::endl;
			break;
	}

	cout << "Opening video device " << cam_id << endl;

    //open the video stream and make sure it's opened
    if( !camera.open(cam_id) )
	{
        std::cout << "Error opening the camera. May be invalid device id. EXIT program." << std::endl;
        return -1;
    }

//	Load the Cascades
		if( !face_cascade.load( face_cascade_name ) ){ printf("--(!)Error loading face cascade\n"); return -1; };

		while (ros::ok())
		    {
				geometry_msgs::Point p;

				bool bSuccess = camera.read(frame);

			    if (!bSuccess) //if not success, break loop
			    {
				   cout << "Cannot read the frame from video file" << endl;
				   break;
				}

			    Point pnt = detectFace(frame);
			    if((pnt.x!=0)&&(pnt.y!=0)){
			    	prevPoint.x=pnt.x;
			    	prevPoint.y=pnt.y;
				}

			    p.x = prevPoint.x;
			    p.y = prevPoint.y;

				point_pub.publish(p);

				ros::spinOnce();
				loop_rate.sleep();

		        int c = waitKey(10);
		        if( (char)c == 27 ) { break; } // escape
		    }
	return 0;
}

/** @function detectFacePosition */
Point detectFace(Mat frame)
{
	vector<Rect> faces;
    Mat frame_gray;
    Point nearest_face;
    Point frm_center(frame.cols*0.5,frame.rows*0.5);
    int min_dist=1000;

    cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );

    //-- Detect faces
    face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_DO_CANNY_PRUNING, Size(30, 30) );

    if(faces.size()==1){
    		rectangle(frame, faces[0], cv::Scalar(255,0,0), 2);
			nearest_face.x = faces[0].x + faces[0].width*0.5;
			nearest_face.y = faces[0].y + faces[0].height*0.5;
    }else if(faces.size()>1){
    	for( size_t i = 0; i < faces.size(); i++ ){
    		rectangle(frame, faces[i], cv::Scalar(255,0,0), 2);
    		int distEucl = distEuclidean(faces[i].x + faces[i].width*0.5,frm_center.x,faces[i].y + faces[i].height*0.5,frm_center.y);
    		if(distEucl < min_dist){
    			nearest_face.x = faces[i].x;
    			nearest_face.y = faces[i].y;
    			min_dist = distEucl;
    		}
    	}
    }
    imshow( "test", frame);
    return nearest_face;
}

int distEuclidean(int x1,int y1,int x2,int y2){
	return sqrt(pow(x2-x1,2)+pow(y2-y1,2));
}



