#include "SDK_wrapper.h"
#include <iostream>
#include <visp3/core/vpArray2D.h>
#include <visp3/core/vpTranslationVector.h>

int main()
{  
  // Variables
  double x, y, z;
  double qw, qx, qy, qz;
  int occluded, count = 0;
  std::string filename = "Matrix.yml", header = "frame : ", header_count;

  // Since we will be using this program while connected on WifiVicon, Host is already known.
  std::string host = "192.168.30.1:801";
  char result_msg[64], subject_name[64], root_segment_name[64];
  unsigned int nb_subjects;

  if(isConnected()) // Check if it's already connected.
  {
    std::cout << "Vicon client is already connected !" << std::endl;
    return -1;
  }

  viconsdk_connect(host.c_str(), result_msg, 64);
  if(result_msg != NULL) { // Error message in result_msg.
    std::cout << result_msg << std::endl;
    //delete result_msg; result_msg = NULL;
    return -1;
  }

  // Now, we are connected to Vicon server.
  if(!isConnected()) {
    std::cout << "Not connected..." << std::endl; // Shouldn't occur... Just in case.
    //delete result_msg; result_msg = NULL;
    return -1;
  }

  // Set streaming mode
  viconsdk_setStreamModeToServerPush(result_msg, 64);
  if(result_msg != NULL) // Error message in result_msg.
  {
    std::cout << result_msg << std::endl;
    //delete result_msg; result_msg = NULL;
    return -1;
  }

  // Enable transmission of segment data.
  viconsdk_enableSegmentData(1, result_msg, 64);
  if(result_msg != NULL) // Error message in result_msg.
  {
    std::cout << result_msg << std::endl;
    //delete result_msg; result_msg = NULL;
    return -1;
  }

  std::cout << "Connection established" << std::endl;

  ///////////////////// THE FOLLOWING SECTION SHOULD BE EXECUTED EVERY TIME WE SAVE AN IMAGE /////////////////////
  // Get a frame.
  viconsdk_getFrame(result_msg, 64);
  if(result_msg != NULL)
  {
    std::cout << result_msg << std::endl;
    //delete result_msg; result_msg = NULL;
    return -1;
  }
  count ++;
  header_count = header + std::to_string(count);

  // Get subject counts.
  viconsdk_getSubjectCount(&nb_subjects, result_msg, 64);
  if(nb_subjects == 0)
  {
    std::cout << "No subject found!" << std::endl;
    //delete result_msg; result_msg = NULL;
    return -1;
  }

  // Get the subject and root segment names.
  viconsdk_getSubjectName(0, subject_name, 64, result_msg, 64);
  std::cout << "Found subject with name : " << subject_name << std::endl;
  viconsdk_getSubjectRootSegmentName(subject_name, root_segment_name, 64, result_msg, 64);

  // Get segment position and orientation.
  viconsdk_getSegmentPos(subject_name, root_segment_name, &x, &y, &z, &occluded, result_msg, 64);
  viconsdk_getSegmentQuat(subject_name, root_segment_name, &qw, &qx, &qy, &qz, &occluded, result_msg, 64);

  std::cout << "Position: x = " << x << " y = " << y << " z = " << z << std::endl;

  // Transform pose to ViSP vpHomogeneous Matrix
  vpHomogeneousMatrix vMd(vpTranslationVector(x, y, z), vpQuaternionVector(qx, qy, qz, qw));

  // Save homogeneous matrix to YAML file.
  vpArray2D<double>::saveYAML(filename, vMd);

  return 0;
}
