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
    cout << "Trying to parse calibrationfile: " << filename.c_str() << endl;
  } else {
    cout << "No calibrationfile param was received" << endl;
    return false;
  }
  
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    cout << "Cannot open " << filename.c_str() << endl;
    return false;
  } else {
    fs["CameraMat"] >> cameraMat;
    fs["DistCoeff"] >> distCoeff;
    fs["ImageSize"] >> imageSize;
  }
  parse_camera_info(cameraMat, distCoeff, imageSize, camerainfo_msg);
  return true;
}

void ros_get_params(const ros::NodeHandle &private_nh, int &fps, int &mode,
                    std::string &format, int &timeout) {
  if (private_nh.getParam("fps", fps)) {
    cout << "fps set to " << fps << endl;
  } else {
    fps = 60;
    cout << "No param received, defaulting fps to " << fps << endl;
  }
  if (private_nh.getParam("mode", mode)) {
    cout << "mode set to " << mode << endl;
  } else {
    mode = 0;
    cout << "No param received, defaulting mode to " << mode << endl;
  }
  
  if (private_nh.getParam("format", format)) {
    cout << "format set to " << format.c_str() << endl;
  } else {
    format = "rgb";
    cout << "No param received, defaulting format to " << format.c_str() << endl;
  }

  if (private_nh.getParam("timeout", timeout)) {
    cout << "timeout set to " << timeout << " ms" << endl;
  } else {
    timeout = 1000;
    cout << "No param received, defaulting timeout to " << timeout << " ms" << endl;
  }  
}

// This function configures a number of settings on the camera including offsets 
// X and Y, width, height, and pixel format. These settings must be applied before
// BeginAcquisition() is called; otherwise, they will be read only. Also, it is
// important to note that settings are applied immediately. This means if you plan
// to reduce the width and move the x offset accordingly, you need to apply such
// changes in the appropriate order.
int ConfigureCustomImageSettings(Spinnaker::GenApi::INodeMap& nodeMap) {
    int result = 0;

    cout << endl << endl << "*** CONFIGURING CUSTOM IMAGE SETTINGS ***" << endl << endl;

    try {
      //
      // Apply mono 8 pixel format
      //
      // *** NOTES ***
      // Enumeration nodes are slightly more complicated to set than other
      // nodes. This is because setting an enumeration node requires working
      // with two nodes instead of the usual one. 
      //
      // As such, there are a number of steps to setting an enumeration node: 
      // retrieve the enumeration node from the nodemap, retrieve the desired 
      // entry node from the enumeration node, retrieve the integer value from 
      // the entry node, and set the new value of the enumeration node with
      // the integer value from the entry node.
      //
      // Retrieve the enumeration node from the nodemap
      Spinnaker::GenApi::CEnumerationPtr ptrPixelFormat = nodeMap.GetNode("PixelFormat");
      if (IsAvailable(ptrPixelFormat) && IsWritable(ptrPixelFormat)) {
        // Retrieve the desired entry node from the enumeration node
        Spinnaker::GenApi::CEnumEntryPtr ptrPixelFormatMono8 =
            ptrPixelFormat->GetEntryByName("Mono8");
        if (IsAvailable(ptrPixelFormatMono8) && IsReadable(ptrPixelFormatMono8)) {
          // Retrieve the integer value from the entry node
          int64_t pixelFormatMono8 = ptrPixelFormatMono8->GetValue();

          // Set integer as new value for enumeration node
          ptrPixelFormat->SetIntValue(pixelFormatMono8);

          cout << "Pixel format set to " << ptrPixelFormat->GetCurrentEntry()->GetSymbolic() << "..." << endl;
        } else {
          cout << "Pixel format mono 8 not available..." << endl;
        }
      } else {
        cout << "Pixel format not available..." << endl;
      }
      
      // 
      // Apply minimum to offset X
      //
      // *** NOTES ***
      // Numeric nodes have both a minimum and maximum. A minimum is retrieved
      // with the method GetMin(). Sometimes it can be important to check 
      // minimums to ensure that your desired value is within range.
      //
      Spinnaker::GenApi::CIntegerPtr ptrOffsetX = nodeMap.GetNode("OffsetX");
      if (IsAvailable(ptrOffsetX) && IsWritable(ptrOffsetX)) {
        ptrOffsetX->SetValue(ptrOffsetX->GetMin());
        cout << "Offset X set to " << ptrOffsetX->GetMin() << "..." << endl;
      } else {
        cout << "Offset X not available..." << endl;
      }

      //
      // Apply minimum to offset Y
      // 
      // *** NOTES ***
      // It is often desirable to check the increment as well. The increment
      // is a number of which a desired value must be a multiple of. Certain
      // nodes, such as those corresponding to offsets X and Y, have an
      // increment of 1, which basically means that any value within range
      // is appropriate. The increment is retrieved with the method GetInc().
      //
      Spinnaker::GenApi::CIntegerPtr ptrOffsetY = nodeMap.GetNode("OffsetY");
      if (IsAvailable(ptrOffsetY) && IsWritable(ptrOffsetY)) {
        ptrOffsetY->SetValue(ptrOffsetY->GetMin());
        cout << "Offset Y set to " << ptrOffsetY->GetValue() << "..." << endl;
      } else {
        cout << "Offset Y not available..." << endl;
      }

      //
      // Set maximum width
      //
      // *** NOTES ***
      // Other nodes, such as those corresponding to image width and height, 
      // might have an increment other than 1. In these cases, it can be
      // important to check that the desired value is a multiple of the
      // increment. However, as these values are being set to the maximum,
      // there is no reason to check against the increment.
      //
      Spinnaker::GenApi::CIntegerPtr ptrWidth = nodeMap.GetNode("Width");
      if (IsAvailable(ptrWidth) && IsWritable(ptrWidth)) {
        int64_t widthToSet = ptrWidth->GetMax();

        ptrWidth->SetValue(widthToSet);
        
        cout << "Width set to " << ptrWidth->GetValue() << "..." << endl;
      } else {
        cout << "Width not available..." << endl;
      }

      //
      // Set maximum height
      //
      // *** NOTES ***
      // A maximum is retrieved with the method GetMax(). A node's minimum and
      // maximum should always be a multiple of its increment.
      //
      Spinnaker::GenApi::CIntegerPtr ptrHeight = nodeMap.GetNode("Height");
      if (IsAvailable(ptrHeight) && IsWritable(ptrHeight)) {
        int64_t heightToSet = ptrHeight->GetMax();
        
        ptrHeight->SetValue(heightToSet);
        
        cout << "Height set to " << ptrHeight->GetValue() << "..." << endl << endl;
      } else {
        cout << "Height not available..." << endl << endl;
      }
    } catch (Spinnaker::Exception &e) {
      cout << "Error: " << e.what() << endl;
      result = -1;
    }
    
    return result;
}

#ifdef _DEBUG
// Disables heartbeat on GEV cameras so debugging does not incur timeout errors
int DisableHeartbeat(Spinnaker::CameraPtr pCam,
                     Spinnaker::GenApi::INodeMap& nodeMap,
                     Spinnaker::GenApi::INodeMap& nodeMapTLDevice) {
  cout << "Checking device type to see if we need to disable the camera's heartbeat..."
       << endl << endl;
  //
  // Write to boolean node controlling the camera's heartbeat
  // 
  // *** NOTES ***
  // This applies only to GEV cameras and only applies when in DEBUG mode.
  // GEV cameras have a heartbeat built in, but when debugging applications the
  // camera may time out due to its heartbeat. Disabling the heartbeat prevents 
  // this timeout from occurring, enabling us to continue with any necessary debugging.
  // This procedure does not affect other types of cameras and will prematurely exit
  // if it determines the device in question is not a GEV camera. 
  //
  // *** LATER ***
  // Since we only disable the heartbeat on GEV cameras during debug mode, it is better
  // to power cycle the camera after debugging. A power cycle will reset the camera 
  // to its default settings. 
  // 
  
  Spinnaker::GenApi::CEnumerationPtr ptrDeviceType = nodeMapTLDevice.GetNode("DeviceType");
  if (!IsAvailable(ptrDeviceType) && !IsReadable(ptrDeviceType)) {
    cout << "Error with reading the device's type. Aborting..." << endl << endl;
    return -1;
  } else {
    if (ptrDeviceType->GetIntValue() == DeviceType_GEV) {
      cout << "Working with a GigE camera. Attempting to disable heartbeat before continuing..."
           << endl << endl;
      Spinnaker::GenApi::CBooleanPtr ptrDeviceHeartbeat =
          nodeMap.GetNode("GevGVCPHeartbeatDisable");
      if (!IsAvailable(ptrDeviceHeartbeat) || !IsWritable(ptrDeviceHeartbeat)) {
        cout << "Unable to disable heartbeat on camera."
            "Continuing with execution as this may be non-fatal..." << endl << endl;
      } else {
        ptrDeviceHeartbeat->SetValue(true);
        cout << "WARNING: Heartbeat on GigE camera disabled for the rest of Debug Mode."
             << endl;
        cout << "         Power cycle camera when done debugging to re-enable the heartbeat..."
             << endl << endl;
      }
    } else {
      cout << "Camera does not use GigE interface. Resuming normal execution..."
           << endl << endl;
    }
  }
  return 0;
}
#endif

int PrintDeviceInfo(Spinnaker::GenApi::INodeMap & nodeMap, std::string camSerial) {
  int result = 0;
  
  cout << "[" << camSerial << "] Printing device information ..." << endl << endl;
  
  Spinnaker::GenApi::FeatureList_t features;
  Spinnaker::GenApi::CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
  if (IsAvailable(category) && IsReadable(category)) {
    category->GetFeatures(features);
    
    Spinnaker::GenApi::FeatureList_t::const_iterator it;
    for (it = features.begin(); it != features.end(); ++it) {
      Spinnaker::GenApi::CNodePtr pfeatureNode = *it;
      Spinnaker::GenApi::CValuePtr pValue = (Spinnaker::GenApi::CValuePtr)pfeatureNode;
      cout << "[" << camSerial << "] " << pfeatureNode->GetName() << " : "
           << (IsReadable(pValue) ? pValue->ToString() : "Node not readable") << endl;
    }
  } else {
    cout << "[" << camSerial << "] " << "Device control information not available." << endl;
  }
    
  cout << endl;
  
  return result;
}

int AcquireImages(Spinnaker::CameraPtr pCam, Spinnaker::GenApi::INodeMap& nodeMap, 
                  std::string serialNumber, ros::Publisher& publisher, int fps) {
  int result = 0;
  cout << "*** IMAGE ACQUISITION ***" << endl;

  try {
    while (running_ && ros::ok()) {
      const unsigned int k_numImages = (unsigned int)fps;
      ros::Rate loop_rate(fps);
      int count = 0;
      for (unsigned int imageCnt = 0; imageCnt < k_numImages; imageCnt++) {        
        try {
          // Retrieve the next received image
          Spinnaker::ImagePtr pResultImage = pCam->GetNextImage();
          
          if (pResultImage->IsIncomplete()) {
            cout << "[" << serialNumber << "] " << "Image incomplete with image status "
                 << pResultImage->GetImageStatus() << "..." << endl << endl;
          } else {
            int cvFormat = CV_8UC3;
            Spinnaker::ImagePtr convertedImage =
                pResultImage->Convert(Spinnaker::PixelFormat_RGB8 ,Spinnaker::HQ_LINEAR);
            
            unsigned int XPadding = convertedImage->GetXPadding();
            unsigned int YPadding = convertedImage->GetYPadding();
            unsigned int rowsize = convertedImage->GetWidth();
            unsigned int colsize = convertedImage->GetHeight();
            
            cv::Mat cvMat = cv::Mat(colsize + YPadding,rowsize + XPadding, cvFormat,
                                    convertedImage->GetData(), convertedImage->GetStride());

            //
            // Public Camera Message
            //
            sensor_msgs::ImagePtr msg;
            std_msgs::Header header;
            msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", cvMat).toImageMsg();
            msg->header.frame_id = "camera";
            msg->header.stamp.sec = ros::Time::now().sec;
            msg->header.stamp.nsec = ros::Time::now().nsec;
            msg->header.seq = count;
            count++;
            
            publisher.publish(msg);

            // Release image
            pResultImage->Release();
          }
        } catch (Spinnaker::Exception &e) {
          cout << "[" << serialNumber << "] " << "Error: " << e.what() << endl;
          result = -1;
        }
      }/* end for */
      loop_rate.sleep();
      ros::spinOnce();
    }/* end while */

    result = 0;
  } catch (Spinnaker::Exception &e) {
    cout << "[" << serialNumber << "] " << "Error: " << e.what() << endl;
    result = -1;
  }
  return result;
}

int RunSingleCamera(Spinnaker::CameraPtr pCam, ros::Publisher& publisher, int fps) {
  int result = 0;
  
  try {
    // Retrieve TL device nodemap and print device information
    Spinnaker::GenApi::INodeMap & nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

    // Retrieve device serial number for filename
    Spinnaker::GenApi::CStringPtr ptrStringSerial =
        pCam->GetTLDeviceNodeMap().GetNode("DeviceSerialNumber");

    std::string serialNumber = "";

    if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial)) {
      serialNumber = ptrStringSerial->GetValue();
    }

    cout << endl << "[" << serialNumber << "] " << "*** IMAGE ACQUISITION THREAD STARTING"
         << " ***" << endl << endl;

    // Print device information
    PrintDeviceInfo(nodeMapTLDevice, serialNumber);

    // Initialize camera
    pCam->Init();

#ifdef _DEBUG
    cout << endl << endl << "*** DEBUG ***" << endl << endl;

    // If using a GEV camera and debugging,
    // should disable heartbeat first to prevent further issues
    if (DisableHeartbeat(pCam, pCam->GetNodeMap(), pCam->GetTLDeviceNodeMap()) != 0) {
      return (void*)0;
    }

    cout << endl << endl << "*** END OF DEBUG ***" << endl << endl;
#endif

    // Set acquisition mode to continuous
    Spinnaker::GenApi::CEnumerationPtr ptrAcquisitionMode =
        pCam->GetNodeMap().GetNode("AcquisitionMode");
    if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode)) {
      cout << "Unable to set acquisition mode to continuous (node retrieval; camera "
           << serialNumber << "). Aborting..." << endl << endl;
      return -1;
    }
    
    Spinnaker::GenApi::CEnumEntryPtr ptrAcquisitionModeContinuous =
        ptrAcquisitionMode->GetEntryByName("Continuous");
    if (!IsAvailable(ptrAcquisitionModeContinuous) ||
        !IsReadable(ptrAcquisitionModeContinuous)) {
      cout << "Unable to set acquisition mode to continuous (entry 'continuous' retrieval "
           << serialNumber << "). Aborting..." << endl << endl;
      return -1;
    }

    int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();
    ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);
    
    cout << "[" << serialNumber << "] " << "Acquisition mode set to continuous..." << endl;

    // Begin acquiring images
    pCam->BeginAcquisition();
    
    cout << "[" << serialNumber << "] " << "Started acquiring images..." << endl;
    
    // Retrieve GenICam nodemap
    Spinnaker::GenApi::INodeMap & nodeMap = pCam->GetNodeMap();

    // Configure custom image settings
    result = ConfigureCustomImageSettings(nodeMap);
    if (result < 0)
      return result;
    
    // Acquire images and save into vector
    result = AcquireImages(pCam, nodeMap, serialNumber, publisher, fps);

    // End acquisition
    pCam->EndAcquisition();
    // Deinitialize camera
    pCam->DeInit();
  } catch (Spinnaker::Exception &e) {
    cout << "Error: " << e.what() << endl;
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
  char result[32] = {0};
  int index = ((thread_args_t *)args)->index;
  int fps = ((thread_args_t *)args)->fps;
  ros::Publisher* publishers_camera_ptr = ((thread_args_t *)args)->publishers_camera_ptr;
  Spinnaker::CameraPtr camera_ptr = ((thread_args_t *)args)->camera_ptr;
  int err = RunSingleCamera(camera_ptr, *publishers_camera_ptr, fps);
  sprintf(result, "RunSingleCamera %d, return: %d", index, err);
  pthread_exit(result);
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
  char* result = "ok";

  try {
    // advertise each camera topic
    for (unsigned int i = 0; i < num_cameras; i++) {
      cout << "Running camera: " << i << endl;
      std::string current_topic = "camera" + std::to_string(i) + "/image_raw";
      publishers_cameras[i] = node_handle.advertise<sensor_msgs::Image>(current_topic, 100);
      camera_threads_args[i] = { &(publishers_cameras[i]), camera_list_ptr->GetByIndex(i),
                                 fps, i };
      int err = pthread_create(&camera_threads[i], NULL,
                               &camera_thread, &camera_threads_args[i]);
      assert(err == 0);
    }

    while (running_ && ros::ok()) {
      sleep(1);
    }
  
    // wait all threads
    for (unsigned int i = 0; i < num_cameras; i++) {
      void* exitcode;
      int rc = pthread_join(camera_threads[i], &exitcode);
      if (rc != 0) {
        cout << "Handle error from pthread_join returned for camera at index " << i << endl;
      } else if ((int)(intptr_t)exitcode == 0) {
        cout << "Grab thread for camera at index " << i << " exited with errors."
            "Please check onscreen print outs for error details" << endl;
      }
    }
  } catch (Spinnaker::Exception &e) {
    cout << "Error: " << e.what() << endl;
    result = "Exception happend";
  }

  cout << "Control Thread exit..." << endl;
  pthread_exit(result);
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
  cout << "Spinnaker library version: "
       << spinnakerLibraryVersion.major << "."
       << spinnakerLibraryVersion.minor << "."
       << spinnakerLibraryVersion.type << "."
       << spinnakerLibraryVersion.build << endl << endl;
  
  // Retrieve list of cameras from the system
  Spinnaker::CameraList camList = system->GetCameras();
  unsigned int numCameras = camList.GetSize();
  
  cout << "Number of cameras detected: " << numCameras << endl;
  
  // Finish if there are no cameras
  if (numCameras == 0) {
    // Clear camera list before releasing system
    camList.Clear();

    // Release system
    system->ReleaseInstance();
    
    cout << "Not enough cameras!" << endl;
    return -1;
  }

  // ------------------------------
  // Now get camera list
  // ------------------------------
  
  // calibration data
  sensor_msgs::CameraInfo camerainfo_msg;
  if (get_matrices_from_file(private_nh, camerainfo_msg)) {
    ros::Publisher camera_info_pub[numCameras];
  }
  
  // create main control thread
  control_thread_args_t thread_args = { &camList, fps };
  pthread_t control_thread_handle;
  int err = pthread_create(&control_thread_handle, nullptr, &control_thread, &thread_args);
  assert(err == 0);

  void* exitcode;
  int rc = pthread_join(control_thread_handle, &exitcode);
  if (rc != 0) {
    cout << "Handle error from pthread_join returned for control thread." << endl;
  } else if ((int)(intptr_t)exitcode == 0) {
    cout << "Grab thread for control thread, exited with errors. "
        "Please check onscreen print outs for error details" << endl;
  }  
  camList.Clear();			        // Clear camera list before releasing system
  system->ReleaseInstance();			// Release system
  
  cout << endl << "Camera node closed correctly" << endl;
  return 0;
}
