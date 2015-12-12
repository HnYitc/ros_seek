//library: cv_bridge, opencv
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//library: seek
#include <thread>
#include <seek.hpp>

using namespace std;
using namespace LibSeek;
using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

class libseek_ros
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  cv_bridge::CvImage *cv_image;
  std_msgs::Header header;

  Imager iface;
  Frame frame;
  int h,w;
  Mat rawImg,showImg;

public:
  libseek_ros()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("/seek_img", 1);

    iface.init();
    iface.frame_init(frame);

    h = frame.height();
    w = frame.width();
    rawImg = Mat::zeros(cv::Size(w,h), CV_16U);
    showImg = Mat::zeros(cv::Size(w,h), CV_8U);

    namedWindow(OPENCV_WINDOW);

  }

  ~libseek_ros()
  {
    destroyWindow(OPENCV_WINDOW);
  }

  void CaptureSeek(){
    iface.frame_acquire(frame);
    h = frame.height();
    w = frame.width();
    //vector<uint16_t> img(w*h);
    {
      int _max = 0x8250;
      int _min = 0x7e50;
      for (int y = 0; y < h; y++) {
        for (int x = 0; x < w-1; x++) {

	  rawImg.at<ushort>(y, x) = frame.data()[y*w+x];
 
          float v = float(frame.data()[y*w+x] - _min) / (_max - _min);

          if (v < 0.0) { v = 0; }
          if (v > 1.0) { v = 1; }

          showImg.at<uchar>(y, x) = (uchar)(v*255.0);

        }
      }
      //medianBlur(showImg, showImg, 3);
      imshow(OPENCV_WINDOW, showImg);

      ///Output modified video stream
      //header.seq = seq_counter;
      //header.frame_id = camera_frame_id_;
      //header.stamp = ros::Time(frame.getTimestamp());
      //cv_image = new cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO16, rawImg);
      cv_image = new cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO16, rawImg);
      image_pub_.publish(cv_image->toImageMsg());


      waitKey(1);

     }

  }

};


int main(int argc, char** argv)
{

  ros::init(argc, argv, "ros_seek");
  libseek_ros sk;
  while(ros::ok()){
    sk.CaptureSeek();
  }
  ros::spin();
  return 0;

}
