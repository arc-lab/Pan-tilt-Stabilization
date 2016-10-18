#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/core/core.hpp>
#include "stabilizer/stabilizer.h"
#include <tf/transform_datatypes.h>


const int HORIZONTAL_BORDER_CROP = 20;



Stabilizer::Stabilizer(string image_topic) :
nh_()
{
  image_cache_.clear();

  const char * imu_topic = "/imu/data";
  
  image_sub_ = nh_.subscribe(image_topic, 1, &Stabilizer::image_callback, this);
  imu_sub_ = nh_.subscribe(imu_topic, 1, &Stabilizer::imu_callback, this);

  image_transport::ImageTransport it(nh_);
  // display_pub_ = it.advertise("tracking_display", 1);
  // bbox_pub_ = nh_.advertise<hand_tracker_2d::HandBBox>("/interact/tracking_bbox", 1);

  ROS_INFO("Ready to stabilize, waiting for imu data");
}

void Stabilizer::imu_callback(const sensor_msgs::Imu& msg)
{

  updateImuCache(msg);
  // ROS_INFO("Imu Seq: [%d]", msg->header.seq);
  // ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
}



void Stabilizer::updateImuCache(const sensor_msgs::Imu& msgs_imu) {
  imu_cache_.push_back(msgs_imu);
  cout << "Imu Cache size: " << imu_cache_.size()
       << endl;
}


void Stabilizer::image_callback(const sensor_msgs::Image& msgs_image)
{
  ROS_INFO("call back");
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msgs_image, sensor_msgs::image_encodings::BGR8);
    curr_1 = cv_ptr->image;
    curr_ = Mat::zeros( curr_1.size(), curr_1.type() );
    for( int y = 0; y < curr_1.rows; y++ )
    { for( int x = 0; x < curr_1.cols; x++ )
         { for( int c = 0; c < 3; c++ )
              {
      curr_.at<Vec3b>(y,x)[c] =
         saturate_cast<uchar>( 1.5*( curr_1.at<Vec3b>(y,x)[c] ) + 70 );
             }
    }
    }
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (image_cache_.size() > 0 and imu_cache_.size() > 0 ){

    sensor_msgs::Image image_tmp = image_cache_.back();
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_tmp, sensor_msgs::image_encodings::BGR8);
    prev_1 = cv_ptr->image;
    prev_ = Mat::zeros( prev_1.size(), prev_1.type() );
    for( int y = 0; y < prev_1.rows; y++ )
    { for( int x = 0; x < prev_1.cols; x++ )
         { for( int c = 0; c < 3; c++ )
              {
      prev_.at<Vec3b>(y,x)[c] =
         saturate_cast<uchar>( 1.5*(prev_1.at<Vec3b>(y,x)[c] ) + 70 );
             }
    }
    }

    sensor_msgs::Imu& imu_msg =imu_cache_.back();
    sensor_msgs::Imu& imu_msg_curr = imu_cache_.back();
    sensor_msgs::Imu& imu_msg_prev = imu_cache_.front();
    // ROS_INFO("Imu Seq: [%d]", imu_msg.header.seq);
    imu_msg.orientation.x = imu_msg_curr.orientation.x - imu_msg_prev.orientation.x;
    imu_msg.orientation.y = imu_msg_curr.orientation.y - imu_msg_prev.orientation.y;
    imu_msg.orientation.z = imu_msg_curr.orientation.z - imu_msg_prev.orientation.z;
    imu_msg.orientation.w = imu_msg_curr.orientation.w - imu_msg_prev.orientation.w;
    ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]",imu_msg.orientation.x,imu_msg.orientation.y,imu_msg.orientation.z,imu_msg.orientation.w);
    Mat T;
    int k;
    k = 100;
    T = Mat(2,3,CV_64F);


    tf::Quaternion q_prev(imu_msg_prev.orientation.x, imu_msg_prev.orientation.y, imu_msg_prev.orientation.z, imu_msg_prev.orientation.w);
    tf::Matrix3x3 m_prev(q_prev);
    double roll_prev, pitch_prev, yaw_prev;
    m_prev.getRPY(roll_prev, pitch_prev, yaw_prev);



    tf::Quaternion q_curr(imu_msg_curr.orientation.x, imu_msg_curr.orientation.y, imu_msg_curr.orientation.z, imu_msg_curr.orientation.w);
    tf::Matrix3x3 m_curr(q_curr);
    double roll_curr, pitch_curr, yaw_curr;
    m_curr.getRPY(roll_curr, pitch_curr, yaw_curr);
    // std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
    // roll = 0;
    // pitch = 0;
    // yaw = 0;
    double roll, pitch, yaw;
    roll  = roll_curr - roll_prev;
    pitch = pitch_curr - pitch_prev;
    yaw = yaw_curr - yaw_prev;
    std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;

    double x,y;
    //yaw is roll
    //roll is yaw
    x = (k*yaw_curr)/2;
    y = (k*pitch_curr)/2;
    T.at<double>(0,0) = cos(0);
    T.at<double>(0,1) = -sin(0);
    T.at<double>(1,0) = sin(0);
    T.at<double>(1,1) = cos(0);
    T.at<double>(0,2) = x;
    T.at<double>(1,2) = y;


    
    int vert_border = HORIZONTAL_BORDER_CROP * prev_.rows / prev_.cols;
    
    warpAffine(prev_, cur2, T, curr_.size());

     cur2 = cur2(Range(vert_border, cur2.rows-vert_border), Range(HORIZONTAL_BORDER_CROP, cur2.cols-HORIZONTAL_BORDER_CROP));

    // Resize cur2 back to cur size, for better side by side comparison
    resize(cur2, cur2, curr_.size());
    // cvtColor( cur2, cur2, CV_BGR2GRAY );
    // equalizeHist( cur2,cur2);

    // Now draw the original and stablised side by side for coolness
    Mat canvas = Mat::zeros(curr_.rows, curr_.cols*2+10, curr_.type());

    curr_.copyTo(canvas(Range::all(), Range(0, cur2.cols)));
    cur2.copyTo(canvas(Range::all(), Range(cur2.cols+10, cur2.cols*2+10)));

    // If too big to fit on the screen, then scale it down by 2, hopefully it'll fit :)
    // if(canvas.cols > 1920) {
    //   resize(canvas, canvas, Size(canvas.cols/2, canvas.rows/2));
    // }
    //outputVideo<<canvas;
    imshow("before and after", canvas);

    waitKey(10);


  }
  updateImageCache(msgs_image);
}

void Stabilizer::updateImageCache(const sensor_msgs::Image& msgs_image) {
  image_cache_.push_back(msgs_image);
  cout << "Image Cache size: " << image_cache_.size()
       << endl;
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "image_listener");
  // ros::NodeHandle nh;
  // cv::namedWindow("view");
  // cv::startWindowThread();
  const char * image_topic = "/dvs/image_raw";
  Stabilizer imustabilizer(image_topic);
  ros::spin();
}