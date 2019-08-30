#include <iostream>
#include <sstream>
#include <vector>

#include <pthread.h>

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

static volatile int running_ = 1;

static void signal_handler(int) {
  running_ = 0;
  ros::shutdown();
  ROS_INFO("get exit signal");
}

void parse_camera_info(const cv::Mat &camMat,
                       const cv::Mat &disCoeff,
                       const cv::Size &imgSize,
                       sensor_msgs::CameraInfo &msg) {
  msg.header.frame_id = "camera";
  
  msg.height = imgSize.height;
  msg.width = imgSize.width;
  
  for (int row = 0; row < 3; row++)
    for (int col = 0; col < 3; col++)
      msg.K[row * 3 + col] = camMat.at<double>(row, col);

  for (int row = 0; row < 3; row++)
    for (int col = 0; col < 4; col++)
      if (col == 3)
        msg.P[row * 4 + col] = 0.0f;
      else
        msg.P[row * 4 + col] = camMat.at<double>(row, col);
  
  for (int row = 0; row < disCoeff.rows; row++)
    for (int col = 0; col < disCoeff.cols; col++)
      msg.D.push_back(disCoeff.at<double>(row, col));
}

bool get_matrices_from_file(const ros::NodeHandle &nh,
                            sensor_msgs::CameraInfo &camerainfo_msg) {
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
    return false;
  }
  
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    ROS_INFO("Cannot open %s", filename.c_str());
    return false;
  } else {
    fs["CameraMat"] >> cameraMat;
    fs["DistCoeff"] >> distCoeff;
    fs["ImageSize"] >> imageSize;
  }
  parse_camera_info(cameraMat, distCoeff, imageSize, camerainfo_msg);
  return true;
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
                    std::string &format, int &timeout) {
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
        ROS_INFO(Spinnaker::GenApi::IsReadable(pValue) ?
                 pValue->ToString() : "Node not readable");
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
  ROS_INFO("*** IMAGE ACQUISITION ***");
  
  try {
    // Set acquisition mode to continuous
    Spinnaker::GenApi::CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
    if (!Spinnaker::GenApi::IsAvailable(ptrAcquisitionMode) || 
        !Spinnaker::GenApi::IsWritable(ptrAcquisitionMode)) {
      ROS_ERROR("Unable to set acquisition mode to continuous (node retrieval). Aborting...");
      return -1;
    }
    
    Spinnaker::GenApi::CEnumEntryPtr ptrAcquisitionModeContinuous =
        ptrAcquisitionMode->GetEntryByName("Continuous");
    if (!Spinnaker::GenApi::IsAvailable(ptrAcquisitionModeContinuous) || 
        !Spinnaker::GenApi::IsReadable(ptrAcquisitionModeContinuous)) {
      ROS_ERROR("Unable to set acquisition mode to continuous. Aborting...");
      return -1;
    }
    
    int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();
    ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

    ROS_INFO("Acquisition mode set to continuous...");
    //int64_t pixelFormat = pCam->PixelFormat.GetValue();

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
            ROS_WARN("Image incomplete with image status %d ...",
                     pResultImage->GetImageStatus());
          } else {
            //ROS_INFO("Grabbed image %d, width = %d, height = ",
            //         imageCnt, pResultImage->GetWidth(), pResultImage->GetHeight());
            // Deep copy image into image vector
            int cvFormat = CV_8UC3;
            Spinnaker::ImagePtr convertedImage =
                pResultImage->Convert(Spinnaker::PixelFormat_RGB8 ,Spinnaker::HQ_LINEAR);
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
    //Spinnaker::GenApi::INodeMap & nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
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

typedef struct _thread_args_t {
  ros::Publisher* publishers_camera_ptr;
  Spinnaker::CameraPtr camera_ptr;
  int fps;
  unsigned int index;
} thread_args_t;

void* camera_thread(void* args) {
  int index = ((thread_args_t *)args)->index;
  int fps = ((thread_args_t *)args)->fps;
  ros::Publisher* publishers_camera_ptr = ((thread_args_t *)args)->publishers_camera_ptr;
  Spinnaker::CameraPtr camera_ptr = ((thread_args_t *)args)->camera_ptr;
  int result = RunSingleCamera(camera_ptr, *publishers_camera_ptr, fps);
  ROS_INFO("RunSingleCamera return %d", result);
  pthread_exit(NULL);
  ROS_INFO("Thread(%d) exit...", index);
}

typedef struct _control_thread_args_t {
  Spinnaker::CameraList* camera_list_ptr;
  int fps;
} control_thread_args_t;

void* control_thread(void *args) {
  int fps = ((control_thread_args_t *)args)->fps;
  Spinnaker::CameraList* camera_list_ptr = ((control_thread_args_t *)args)->camera_list_ptr;
  unsigned int num_cameras = camera_list_ptr->GetSize();

  ros::NodeHandle node_handle;
  ros::Publisher publishers_cameras[num_cameras];
  pthread_t camera_threads[num_cameras];
  thread_args_t camera_threads_args[num_cameras];

  ROS_INFO("Publishing...");
  // advertise each camera topic
  for (unsigned int i = 0; i < num_cameras; i++) {
    ROS_INFO("Running camera: %d", i);
    std::string current_topic = "camera" + std::to_string(i) + "/image_raw";
    publishers_cameras[i] = node_handle.advertise<sensor_msgs::Image>(current_topic, 100);
    camera_threads_args[i] = { &(publishers_cameras[i]), camera_list_ptr->GetByIndex(i),
                               fps, i };
    pthread_create(&camera_threads[i], NULL,
                   camera_thread, (void*)&(camera_threads_args[i]));
  }

  while (running_ && ros::ok()) {
    sleep(1);
  }
  
  // wait all threads
  for (unsigned int i = 0; i < num_cameras; i++) {
    pthread_join(camera_threads[i], NULL);
  }
  pthread_exit(NULL);
  ROS_INFO("Control Thread exit...");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "bfly_node");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");
  
  signal(SIGTERM, signal_handler); //detect closing
  int fps = 0, camera_mode = 0, timeout = 0;
  std::string format;
  ros_get_params(private_nh, fps, camera_mode, format, timeout);
  
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
  Spinnaker::CameraList camera_list = system->GetCameras();
  unsigned int num_cameras = camera_list.GetSize();
  
  ROS_INFO("Number of cameras detected: %d", num_cameras);
  
  // Finish if there are no cameras
  if (num_cameras == 0) {
    // Clear camera list before releasing system
    camera_list.Clear();

    // Release system
    system->ReleaseInstance();
    ROS_ERROR("Couldn't find BFLY camera");
    return -1;
  }

  // calibration data
  sensor_msgs::CameraInfo camerainfo_msg;
  if (get_matrices_from_file(private_nh, camerainfo_msg)) {
    ros::Publisher camera_info_pub[num_cameras];
  }
  

  control_thread_args_t thread_args = { &camera_list, fps };
  pthread_t control_thread_handle;
  pthread_create(&control_thread_handle, NULL,
                 control_thread, (void*)&(thread_args));
  pthread_join(control_thread_handle, NULL);
  
  camera_list.Clear();			        // Clear camera list before releasing system
  system->ReleaseInstance();			// Release system
  
  ROS_INFO("Camera node closed correctly");
  return 0;
}
