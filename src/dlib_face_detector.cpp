// Copyright 2014 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include "rclcpp/rclcpp.hpp"
//#include "rcutils/cmdline_parser.h"
#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"


using namespace std::chrono_literals;
using namespace std;
using namespace dlib;



int main(int argc, char * argv[])
{
  double freq = 100;
  // Pass command line arguments to rclcpp.
  rclcpp::init(argc, argv);

  //define topic name
  std::string topic("face");

  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within a launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Initialize a ROS 2 node to publish locations of detected faces in an image
  auto node = rclcpp::Node::make_shared("face_detector");
  rclcpp::Logger node_logger = node->get_logger();

  RCLCPP_INFO(node_logger, "Publishing data on topic '%s'", topic.c_str());
  // Create the image publisher with our custom QoS profile.
  auto pub = node->create_publisher<std_msgs::msg::Float64>(topic);

  // Set a loop rate for our main event loop.
  rclcpp::WallRate loop_rate(freq);

  // open cv opject 
  cv::VideoCapture cap;
  cap.open(0);

  unsigned int low_width = 256 ;
  unsigned int low_height= 256 ;
  unsigned int medium_width = 640 ;
  unsigned int medium_height= 480 ;
  unsigned int high_width = 1216 ;
  unsigned int high_height= 912 ;


   #if CV_MAJOR_VERSION < 3
    cap.set(CV_CAP_PROP_FRAME_WIDTH, static_cast<double>(width));
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, static_cast<double>(height));
   #else
    cap.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(medium_width));
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(medium_height));
   #endif

  // dlib ojects
  frontal_face_detector detector = get_frontal_face_detector();
  image_window win;



  // Initialize OpenCV image matrices.
  cv::Mat frame;


  // Initialize a shared pointer to an Image message.
  auto msg = std::make_shared<std_msgs::msg::Float64>();
  
  size_t i = 1;
  // Our main event loop will spin until the user presses CTRL-C to exit.
  while (rclcpp::ok()) {

    // Get the frame from the video capture.
    cap.read(frame);
    
    cv_image<bgr_pixel> cimg(frame);

    // Detect faces 
    std::vector<rectangle> faces = detector(cimg);


    win.clear_overlay();
    win.set_image(cimg);
    win.add_overlay(faces);
    
    //for (auto  it = faces.begin() ; it != faces.end(); ++it)
    //{
    //  cout << it[0] << endl;
    //}
    double myVar = 0 ; 
    if(faces.size()!=0)
    {
    auto  it = faces.begin();
     myVar = it[0].br_corner().x();
    }

    msg->data = myVar;

    //RCLCPP_INFO(node_logger, "Publishing image #%zd", faces.size());
    pub->publish(msg);


    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
