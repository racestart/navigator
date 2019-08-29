#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <signal.h>

#include <opencv2/opencv.hpp>
#include <spinnaker/Spinnaker.h>
#include <spinnaker/SpinVideo.h>
#include <spinnaker/SpinGenApi/SpinnakerGenApi.h>
#include <spinnaker/CameraDefs.h>
#include <spinnaker/spinc/SpinnakerDefsC.h>

#include "std_msgs/String.h"

using namespace std;
using namespace cv;
using namespace std;

static volatile int running_ = 1;

static void signal_handler(int) {
  running_ = 0;
  ros::shutdown();
}

void parse_camera_info(const cv::Mat &camMat,
                       const cv::Mat &disCoeff,
                       const cv::Size &imgSize,
                       sensor_msgs::CameraInfo &msg) {
  msg.header.frame_id = "camera";
  
  msg.height = imgSize.height;
  msg.width = imgSize.width;
  
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
      msg.K[row * 3 + col] = camMat.at<double>(row, col);
    }
  }

  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 4; col++) {
      if (col == 3) {
        msg.P[row * 4 + col] = 0.0f;
      } else {
        msg.P[row * 4 + col] = camMat.at<double>(row, col);
      }
    }
  }
  
  for (int row = 0; row < disCoeff.rows; row++) {
    for (int col = 0; col < disCoeff.cols; col++) {
      msg.D.push_back(disCoeff.at<double>(row, col));
    }
  }
}

/*!
 * Reads and parses the Autoware calibration file format
 * @param nh ros node handle
 * @param camerainfo_msg CameraInfo message to fill
 */
void getMatricesFromFile(const ros::NodeHandle &nh, sensor_msgs::CameraInfo &camerainfo_msg) {
  //////////////////CAMERA INFO/////////////////////////////////////////
  cv::Mat cameraExtrinsicMat;
  cv::Mat cameraMat;
  cv::Mat distCoeff;
  cv::Size imageSize;
  std::string filename;
  
  if (nh.getParam("calibrationfile", filename) && filename != "") {
    ROS_INFO("Trying to parse calibrationfile :");
    ROS_INFO("> %s", filename.c_str());
  } else {
    ROS_INFO("No calibrationfile param was received");
    return;
  }
  
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    ROS_INFO("Cannot open %s", filename.c_str());
    return;
  } else {
    fs["CameraMat"] >> cameraMat;
    fs["DistCoeff"] >> distCoeff;
    fs["ImageSize"] >> imageSize;
  }
  parse_camera_info(cameraMat, distCoeff, imageSize, camerainfo_msg);
}

/*!
 * Reads the params from the console
 * @param private_nh[in] Private Ros node handle
 * @param fps[out] Read value from the console double
 * @param mode[out] Read value from the console integer
 * @param format[out] Read value from the console mono or rgb
 * @param timeout[out] Read value from the console timeout in ms
 */
void ros_get_params(const ros::NodeHandle &private_nh, int &fps, int &mode, 
                    std::string &format, int &timeout, int &camera_selected) {
  if (private_nh.getParam("fps", fps)) {
    ROS_INFO("fps set to %d", fps);
  } else {
    fps = 60;
    ROS_INFO("No param received, defaulting fps to %d", fps);
  }
  if (private_nh.getParam("mode", mode)) {
    ROS_INFO("mode set to %d", mode);
  } else {
    mode = 0;
    ROS_INFO("No param received, defaulting mode to %d", mode);
  }
  
  if (private_nh.getParam("format", format)) {
    ROS_INFO("format set to %s", format.c_str());
  } else {
    format = "rgb";
    ROS_INFO("No param received, defaulting format to %s", format.c_str());
  }

  if (private_nh.getParam("timeout", timeout)) {
    ROS_INFO("timeout set to %d ms", timeout);
  } else {
    timeout = 1000;
    ROS_INFO("No param received, defaulting timeout to %d ms", timeout);
  }
  
  if (private_nh.getParam("camera_selected", camera_selected)) {
    ROS_INFO("camera_selected set to %d ms", camera_selected);
  } else {
    camera_selected = 0;
    ROS_INFO("No param received, defaulting camera to %d index", camera_selected);
  }
}

int PrintDeviceInfo(Spinnaker::GenApi::INodeMap & nodeMap) {
  int result = 0;
  ROS_INFO("*** DEVICE INFORMATION ***");
  
  try {
    Spinnaker::GenApi::FeatureList_t features;
    Spinnaker::GenApi::CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
    if (Spinnaker::GenApi::IsAvailable(category) && Spinnaker::GenApi::IsReadable(category)) {
      category->GetFeatures(features);
      
      Spinnaker::GenApi::FeatureList_t::const_iterator it;
      for (it = features.begin(); it != features.end(); ++it) {
        Spinnaker::GenApi::CNodePtr pfeatureNode = *it;
        ROS_INFO("%s : ", pfeatureNode->GetName());
        Spinnaker::GenApi::CValuePtr pValue = (Spinnaker::GenApi::CValuePtr)pfeatureNode;
        ROS_INFO(Spinnaker::GenApi::IsReadable(pValue) ? pValue->ToString() : "Node not readable");
      }
    } else {
      ROS_WARN("Device control information not available.");
    }
  } catch (Spinnaker::Exception &e) {
    ROS_ERROR("Spinnaker: %s", e.what());
    result = -1;
  }
  return result;
}

int AcquireImages(Spinnaker::CameraPtr pCam, Spinnaker::GenApi::INodeMap& nodeMap, 
                  ros::Publisher& publisher, int fps) {
  int result = 0;
  static int num_count = 0;    
  ROS_INFO("*** IMAGE ACQUISITION ***");
  
  try {
    // Set acquisition mode to continuous
    Spinnaker::GenApi::CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
    if (!Spinnaker::GenApi::IsAvailable(ptrAcquisitionMode) || 
        !Spinnaker::GenApi::IsWritable(ptrAcquisitionMode)) {
      ROS_ERROR("Unable to set acquisition mode to continuous (node retrieval). Aborting...");
      return -1;
    }
    
    Spinnaker::GenApi::CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
    if (!Spinnaker::GenApi::IsAvailable(ptrAcquisitionModeContinuous) || 
        !Spinnaker::GenApi::IsReadable(ptrAcquisitionModeContinuous)) {
      ROS_ERROR("Unable to set acquisition mode to continuous (entry 'continuous' retrieval). Aborting...");
      return -1;
    }
    
    int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();
    ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

    ROS_INFO("Acquisition mode set to continuous...");
    int64_t pixelFormat = pCam->PixelFormat.GetValue();

    // Begin acquiring images
    pCam->BeginAcquisition();
    ROS_INFO("Acquiring images...");
		
    // Retrieve and convert images
    //for (unsigned int imageCnt = 0; imageCnt < k_numImages; imageCnt++)

    while (running_ && ros::ok()) {
      const unsigned int k_numImages = (unsigned int)fps;
      ros::Rate loop_rate(fps);
      int count = 0;
      for (unsigned int imageCnt = 0; imageCnt < k_numImages; imageCnt++) {
        // Retrieve the next received image
        Spinnaker::ImagePtr pResultImage = pCam->GetNextImage();
        try {
          if (pResultImage->IsIncomplete()) {
            ROS_WARN("Image incomplete with image status %d ...", pResultImage->GetImageStatus());
          } else {
            //ROS_INFO("Grabbed image %d, width = %d, height = ", imageCnt, pResultImage->GetWidth(), pResultImage->GetHeight());
            // Deep copy image into image vector
            int cvFormat = CV_8UC3;
            Spinnaker::ImagePtr convertedImage = pResultImage->Convert(Spinnaker::PixelFormat_RGB8 ,Spinnaker::HQ_LINEAR);
            unsigned int XPadding = convertedImage->GetXPadding();
            unsigned int YPadding = convertedImage->GetYPadding();
            unsigned int rowsize = convertedImage->GetWidth();
            unsigned int colsize = convertedImage->GetHeight();
            
            cv::Mat cvMat = cv::Mat(colsize + YPadding,rowsize + XPadding, cvFormat, 
                                    convertedImage->GetData(), convertedImage->GetStride());
            
            sensor_msgs::ImagePtr msg;
            std_msgs::Header header;
            msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", cvMat).toImageMsg();
            msg->header.frame_id = "camera";
            msg->header.stamp.sec = ros::Time::now().sec;
            msg->header.stamp.nsec = ros::Time::now().nsec;
            msg->header.seq = count;
            count++;
            
            publisher.publish(msg);
          }
        } catch (Spinnaker::Exception &e) {
          ROS_ERROR("Spinnaker: %s", e.what());
          result = -1;
        }

        // Release image
        pResultImage->Release();
      }/* end for */
      loop_rate.sleep();
      ros::spinOnce();
    }/* end while */
    
    // End acquisition
    pCam->EndAcquisition();
  } catch (Spinnaker::Exception &e) {
    ROS_ERROR("Spinnaker: %s", e.what());
    result = -1;
  }
  return result;
}

int RunSingleCamera(Spinnaker::CameraPtr pCam, ros::Publisher& publisher, int fps) {
  int result = 0;
  int err = 0;
  
  try {
    // Retrieve TL device nodemap and print device information
    Spinnaker::GenApi::INodeMap & nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
    //result = PrintDeviceInfo(nodeMapTLDevice);
    // Initialize camera
    pCam->Init();
    
    // Retrieve GenICam nodemap
    Spinnaker::GenApi::INodeMap & nodeMap = pCam->GetNodeMap();
    
    // Acquire images and save into vector
    err = AcquireImages(pCam, nodeMap, publisher, fps);
    if (err < 0) {
      return err;
    }
    // Deinitialize camera
    pCam->DeInit();
  } catch (Spinnaker::Exception &e) {
    ROS_ERROR("Error: %s\r\n", e.what());
    result = -1;
  }
  
  return result;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "bfly_node");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");
  
  signal(SIGTERM, signal_handler); //detect closing
  int fps = 0, camera_mode = 0, timeout = 0, camera_selected = 0;
  std::string format;
  ros_get_params(private_nh, fps, camera_mode, format, timeout, camera_selected);
  
  // Retrieve singleton reference to system object
  Spinnaker::SystemPtr system = Spinnaker::System::GetInstance();
  
  // Print out current library version
  const Spinnaker::LibraryVersion spinnakerLibraryVersion = system->GetLibraryVersion();
  ROS_INFO("Spinnaker library version: %d.%d.%d.%d", 
           spinnakerLibraryVersion.major,
           spinnakerLibraryVersion.minor,
           spinnakerLibraryVersion.type,
           spinnakerLibraryVersion.build);
  
  // Retrieve list of cameras from the system
  Spinnaker::CameraList camList = system->GetCameras();
  unsigned int nCameras = camList.GetSize();
  
  ROS_INFO("Number of cameras detected: %d", nCameras);
  
  // Finish if there are no cameras
  if (nCameras == 0) {
    // Clear camera list before releasing system
    camList.Clear();

    // Release system
    system->ReleaseInstance();
    ROS_ERROR("Couldn't find BFLY camera");
    return -1;
  }

  // calibration data
  sensor_msgs::CameraInfo camerainfo_msg;
  getMatricesFromFile(private_nh, camerainfo_msg);
  
  ros::Publisher publishers_cameras[nCameras];
  ros::Publisher camera_info_pub[nCameras];
  ros::NodeHandle node_handle;
  
  // advertise each camera topic
  for (unsigned int i = 0; i < nCameras; i++) {
    std::string current_topic = "camera" + std::to_string(i) + "/image_raw";
    publishers_cameras[i] = node_handle.advertise<sensor_msgs::Image>(current_topic, 100);
  }

  ROS_INFO("Publishing...");
  ROS_INFO("Capturing with \'%d\' BFLY cameras", nCameras);
  
  ROS_INFO("Running camera: %d", camera_selected);
  int result = RunSingleCamera(camList.GetByIndex(camera_selected), publishers_cameras[camera_selected], fps);
  camList.Clear();			        // Clear camera list before releasing system
  system->ReleaseInstance();			// Release system
  
  ROS_INFO("Camera node closed correctly");
  return 0;
}
