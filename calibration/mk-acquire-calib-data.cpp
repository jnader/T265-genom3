#include <iostream>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/sensor/vpRealSense2.h>

#include "SDK_wrapper.h"

#if defined(VISP_HAVE_REALSENSE2) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && defined(VISP_HAVE_X11)

bool vicon_connect(const std::string &host)
{
  char result_msg[64];

  if(isConnected()) // Check if it's already connected.
  {
    std::cout << "Vicon client is already connected !" << std::endl;
    return false;
  }

  if(viconsdk_connect(host.c_str(), result_msg, 64)) {
//  if(result_msg != NULL) { // Error message in result_msg.
    std::cout << result_msg << std::endl;
    return false;
  }

  // Now, we are connected to Vicon server.
  if(!isConnected()) {
    std::cout << "Not connected..." << std::endl; // Shouldn't occur... Just in case.
    //delete result_msg; result_msg = NULL;
    return false;
  }

  // Set streaming mode
  if(viconsdk_setStreamModeToServerPush(result_msg, 64))
//  if(result_msg != NULL) // Error message in result_msg.
  {
    std::cout << result_msg << std::endl;
    //delete result_msg; result_msg = NULL;
    return false;
  }

  // Enable transmission of segment data.
  if(viconsdk_enableSegmentData(1, result_msg, 64))
//  if(result_msg != NULL) // Error message in result_msg.
  {
    std::cout << result_msg << std::endl;
    //delete result_msg; result_msg = NULL;
    return false;
  }

  return true;
}

bool vicon_get_pose(vpPoseVector &vPd)
{
  char result_msg[64], subject_name[64], root_segment_name[64];
  unsigned int nb_subjects;
  int occluded;
  double x, y, z;
  double qw, qx, qy, qz;

  // Get a frame.
  if(viconsdk_getFrame(result_msg, 64))
//  if(result_msg != NULL)
  {
    std::cout << result_msg << std::endl;
    //delete result_msg; result_msg = NULL;
    return false;
  }

  // Get subject counts.
  if(viconsdk_getSubjectCount(&nb_subjects, result_msg, 64))
//  if(nb_subjects == 0)
  {
    std::cout << "No subject found!" << std::endl;
    //delete result_msg; result_msg = NULL;
    return false;
  }

  // Get the subject and root segment names.
  viconsdk_getSubjectName(0, subject_name, 64, result_msg, 64);
  std::cout << "Found subject with name : " << subject_name << std::endl;
  viconsdk_getSubjectRootSegmentName(subject_name, root_segment_name, 64, result_msg, 64);

  // Get segment position and orientation.
  viconsdk_getSegmentPos(subject_name, root_segment_name, &x, &y, &z, &occluded, result_msg, 64);
  viconsdk_getSegmentQuat(subject_name, root_segment_name, &qw, &qx, &qy, &qz, &occluded, result_msg, 64);

  x /= 1000.;
  y /= 1000.;
  z /= 1000.;

  std::cout << "Position: x = " << x << " y = " << y << " z = " << z << std::endl;

  // Transform pose to ViSP vpHomogeneous Matrix
  vpHomogeneousMatrix vMd(vpTranslationVector(x, y, z), vpQuaternionVector(qx, qy, qz, qw));
  vPd.buildFrom(vMd);

  return true;
}

int main(int argc, char **argv)
{
  try {
    bool opt_without_vicon = false;
    std::string opt_vicon_host = "192.168.30.1:801";


    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--without-vicon") {
        opt_without_vicon = true;
      }
      else if (std::string(argv[i]) == "--vicon-ip") {
        opt_vicon_host = std::string(argv[i + 1]);
      }
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << "\nUsage: " << std::endl
                  << argv[0]
                  << " [--without-vicon]"
                  << " [--vicon-ip <address:port>]"
                  << " [--help] [-h]\n"
                  << std::endl;
        std::cout << "Options: " << std::endl
                  << "--without-vicon" << std::endl
                  << "\tDoesn't connect to Vicon to get object pose." << std::endl
                  << std::endl
                  << "--vicon-ip <address:port>" << std::endl
                  << "\tVicon router IP address and port. Default: " << opt_vicon_host << std::endl
                  << std::endl
                  << "--help, -h" << std::endl
                  << "\tPrint this helper message" << std::endl
                  << std::endl;
        return EXIT_SUCCESS;
      }
    }

    vpImage<unsigned char> I(500, 500, 255);
    vpHomogeneousMatrix cMo;
    vpDisplayX *display = nullptr;

    vpRealSense2 g;
    rs2::config config;
    config.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    g.open(config);

    bool end = false;
    unsigned cpt = 0;
    bool init_done = false;

    if (! opt_without_vicon) {
      if (vicon_connect(opt_vicon_host)) {
        std::cout << "Vicon connection established" << std::endl;
      }
      else {
        std::cout << "Unable to establish connection with Vicon" << std::endl;
        return EXIT_FAILURE;
      }
    }

    while (! end) {
      g.getOdometryData(&cMo, NULL, NULL, NULL);

      if (! init_done) {
        unsigned int width = I.getWidth();
        unsigned int height = I.getHeight();

        std::cout << "Image size: " << width << " x " << height << std::endl;

        display = new vpDisplayX(I, 10, 10, "Image");

        init_done = true;
        vpDisplay::display(I);
      }

      vpDisplay::displayText(I, 15, 15, "Left click to acquire data", vpColor::red);
      vpDisplay::displayText(I, 30, 15, "Right click to quit", vpColor::red);
      vpMouseButton::vpMouseButtonType button;
      if (vpDisplay::getClick(I, button, false)) {
        if (button == vpMouseButton::button1) {
          cpt ++;

          // Save pose from T265
          std::stringstream ss_img_file, ss_pos_file, ss_msg;
          {
            vpPoseVector cPo(cMo.inverse());
            ss_pos_file << "pose_cPo_" << cpt << ".yaml";
            ss_msg << "Save " << ss_pos_file.str();
            cPo.saveYAML(ss_pos_file.str(), cPo);
            ss_pos_file.str("");
          }

          std::cout << "T265 position: " << cMo.getTranslationVector().t() << std::endl;

          // Save pose from Vicon
          if (! opt_without_vicon) {
            vpPoseVector fPe;
            if (vicon_get_pose(fPe)) {
              ss_pos_file << "pose_fPe_" << cpt << ".yaml";
              ss_msg << " and " << ss_pos_file.str();
              fPe.saveYAML(ss_pos_file.str(), fPe);
              ss_pos_file.str("");
            }
            else {
              std::cout << "Unable to get object pose from Vicon" << std::endl;
              end = true;
            }
          }
          std::cout << ss_msg.str() << std::endl;
        }
        else if (button == vpMouseButton::button3) {
          end = true;
        }
      }
      vpDisplay::flush(I);
    }

    if (display) {
      delete display;
    }
  } catch (const vpException &e) {
    std::cerr << "RealSense error " << e.what() << std::endl;
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
#else
int main()
{
#if !defined(VISP_HAVE_REALSENSE2)
  std::cout << "Install librealsense-2.x." << std::endl;
#endif
#if !(VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::cout << "Build ViSP with c++11 or higher compiler flag (cmake -DUSE_CXX_STANDARD=11)." << std::endl;
#endif

  std::cout << "After installation of the missing 3rd parties, configure ViSP with cmake"
            << " and build ViSP again." << std::endl;
  return 0;
}
#endif
