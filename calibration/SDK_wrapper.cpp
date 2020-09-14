/*
 * Copyright (c) 2017 IRISA/Inria
 * All rights reserved.
 *
 * Redistribution and use  in source  and binary  forms,  with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 *				      Quentin Delamare on Mon Mar 13 2017
 */


/* to be compiled into obj with :
g++ -c SDK_wrapper.cpp -fPIC
*/

#include "ViconSDK.h"
#include "SDK_wrapper.h"

#include <stdio.h>
#include <string.h>

// instanciate the client class once here:
ViconDataStreamSDK::CPP::Client client_cpp;

// convert vicon result::enum to char* error message (inout) and 0/-1 for success/error (returned)
int handle_result(ViconDataStreamSDK::CPP::Result::Enum res, char* emsg, size_t emsgsize) {
  switch(res) {
  case ViconDataStreamSDK::CPP::Result::Success:
    return 0;

  case ViconDataStreamSDK::CPP::Result::InvalidHostName:
    snprintf(emsg, emsgsize, "ViconSDK returned InvalidHostName");
    break;

  case ViconDataStreamSDK::CPP::Result::ClientAlreadyConnected:
    snprintf(emsg, emsgsize, "ViconSDK returned ClientAlreadyConnected");
    break;

  case ViconDataStreamSDK::CPP::Result::ClientConnectionFailed:
    snprintf(emsg, emsgsize, "ViconSDK returned ClientConnectionFailed");
    break;

  case ViconDataStreamSDK::CPP::Result::NotConnected:
    snprintf(emsg, emsgsize, "ViconSDK returned NotConnected");
    break;

  case ViconDataStreamSDK::CPP::Result::NoFrame:
    snprintf(emsg, emsgsize, "ViconSDK returned NoFrame");
    break;

  case ViconDataStreamSDK::CPP::Result::InvalidSubjectName:
    snprintf(emsg, emsgsize, "ViconSDK returned InvalidSubjectName");
    break;

  case ViconDataStreamSDK::CPP::Result::InvalidSegmentName:
    snprintf(emsg, emsgsize, "ViconSDK returned InvalidSegmentName");
    break;

  case ViconDataStreamSDK::CPP::Result::InvalidMarkerName:
    snprintf(emsg, emsgsize, "ViconSDK returned InvalidMarkerName");
    break;

  case ViconDataStreamSDK::CPP::Result::InvalidIndex:
    snprintf(emsg, emsgsize, "ViconSDK returned InvalidIndex");
    break;
    
  default:
    snprintf(emsg, emsgsize, "ViconSDK returned something strange...");
    break;
  }
  return -1;
}


// Let's implement functions that can be used from C, even if some C++ code is used inside
extern "C" {
// the following are simple functions replicating SDK's functionnalities, but without objects/namespaces

int viconsdk_getSubjectCount(unsigned int* out_count, char* emsg, size_t emsgsize) {
  // Vicon SDK defines objects even for output types, let's extract data from it:
  ViconDataStreamSDK::CPP::Output_GetSubjectCount scnt = client_cpp.GetSubjectCount();
  *out_count = scnt.SubjectCount;

  return handle_result(scnt.Result, emsg, emsgsize); // 0:success / -1:error
}

int viconsdk_getMarkerCount(const char* SubjectName, unsigned int* out_count, char* emsg, size_t emsgsize) {
  ViconDataStreamSDK::CPP::Output_GetMarkerCount mcnt = client_cpp.GetMarkerCount(SubjectName);
  *out_count = mcnt.MarkerCount;

  return handle_result(mcnt.Result, emsg, emsgsize); // 0:success / -1:error
}

int viconsdk_getUnlabeledMarkerCount(unsigned int* out_count, char* emsg, size_t emsgsize) {
  ViconDataStreamSDK::CPP::Output_GetUnlabeledMarkerCount mcnt = client_cpp.GetUnlabeledMarkerCount();
  *out_count = mcnt.MarkerCount;

  return handle_result(mcnt.Result, emsg, emsgsize); // 0:success / -1:error
}

int viconsdk_getSubjectName(const unsigned int SubjectIndex, char* out_name, size_t name_size, char* emsg, size_t emsgsize) {
  ViconDataStreamSDK::CPP::Output_GetSubjectName sn = client_cpp.GetSubjectName(SubjectIndex);
  std::string subjectName = sn.SubjectName; // cast to string for c_str(), otherwise Vicon's String doesn't provide it
  strncpy(out_name, subjectName.c_str(), name_size);

  return handle_result(sn.Result, emsg, emsgsize); // 0:success / -1:error
}

/*int viconsdk_getSegmentName(const char* SubjectName, const unsigned int SegmentIndex, const char** out_name, char* emsg, size_t emsgsize) {
    ViconDataStreamSDK::CPP::Output_GetSegmentName sn = client_cpp.GetSegmentName(SubjectName, SegmentIndex);
    std::string segmentName = sn.SegmentName; // same: cast as a string
    *out_name = segmentName.c_str();
    
    return handle_result(sn.Result, emsg, emsgsize); // 0:success / -1:error
  }*/ // not required

int viconsdk_getMarkerName(const char* SubjectName, const unsigned int MarkerIndex, char* out_name, size_t mname_size, char* emsg, size_t emsgsize) {
  ViconDataStreamSDK::CPP::Output_GetMarkerName mn = client_cpp.GetMarkerName(SubjectName, MarkerIndex);
  std::string markerName = mn.MarkerName; // same: cast as a string
  // *out_name = markerName.c_str();
  strncpy(out_name, markerName.c_str(), mname_size);

  return handle_result(mn.Result, emsg, emsgsize); // 0:success / -1:error
}

int viconsdk_getSubjectRootSegmentName(const char* SubjectName, char* out_name, size_t rsname_size, char* emsg, size_t emsgsize) {
  ViconDataStreamSDK::CPP::Output_GetSubjectRootSegmentName rn = client_cpp.GetSubjectRootSegmentName(SubjectName);
  std::string rootSegmentName = rn.SegmentName; // same: cast as a string
  // *out_name = rootSegmentName.c_str();
  strncpy(out_name, rootSegmentName.c_str(), rsname_size);

  return handle_result(rn.Result, emsg, emsgsize); // 0:success / -1:error
}

int viconsdk_getSegmentPos(const char* SubjectName, const char* SegmentName, double* out_x, double* out_y, double* out_z, int* out_occluded, char* emsg, size_t emsgsize) {
  ViconDataStreamSDK::CPP::Output_GetSegmentGlobalTranslation st = client_cpp.GetSegmentGlobalTranslation(SubjectName, SegmentName);
  *out_x = st.Translation[0];
  *out_y = st.Translation[1];
  *out_z = st.Translation[2];

  *out_occluded = st.Occluded;

  return handle_result(st.Result, emsg, emsgsize); // 0:success / -1:error
}

int viconsdk_getSegmentQuat(const char* SubjectName, const char* SegmentName, double* out_qw, double* out_qx, double* out_qy, double* out_qz, int* out_occluded, char* emsg, size_t emsgsize) {
  ViconDataStreamSDK::CPP::Output_GetSegmentGlobalRotationQuaternion sr = client_cpp.GetSegmentGlobalRotationQuaternion(SubjectName, SegmentName);
  *out_qx = sr.Rotation[0];
  *out_qy = sr.Rotation[1];
  *out_qz = sr.Rotation[2];
  *out_qw = sr.Rotation[3];

  *out_occluded = sr.Occluded;

  return handle_result(sr.Result, emsg, emsgsize); // 0:success / -1:error
}

int viconsdk_getMarkerPos(const char* SubjectName, const char* MarkerName, double* out_x, double* out_y, double* out_z, int* out_occluded, char* emsg, size_t emsgsize) {
  ViconDataStreamSDK::CPP::Output_GetMarkerGlobalTranslation mt = client_cpp.GetMarkerGlobalTranslation(SubjectName, MarkerName);
  *out_x = mt.Translation[0];
  *out_y = mt.Translation[1];
  *out_z = mt.Translation[2];

  *out_occluded = mt.Occluded;

  return handle_result(mt.Result, emsg, emsgsize); // 0:success / -1:error
}

int viconsdk_getUnlabeledMarkerPos(const unsigned int MarkerIndex, double* out_x, double* out_y, double* out_z, char* emsg, size_t emsgsize) {
  ViconDataStreamSDK::CPP::Output_GetUnlabeledMarkerGlobalTranslation mt = client_cpp.GetUnlabeledMarkerGlobalTranslation(MarkerIndex);
  *out_x = mt.Translation[0];
  *out_y = mt.Translation[1];
  *out_z = mt.Translation[2];

  return handle_result(mt.Result, emsg, emsgsize); // 0:success / -1:error
}

int viconsdk_connect(const char* host_extended, char* emsg, size_t emsgsize) {
  return handle_result(client_cpp.Connect(host_extended).Result, emsg, emsgsize); // 0:success / -1:error
}

int isConnected() {
  return client_cpp.IsConnected().Connected;
}

int viconsdk_disconnect(char* emsg, size_t emsgsize) {
  return handle_result(client_cpp.Disconnect().Result, emsg, emsgsize); // 0:success / -1:error
}

int viconsdk_setStreamModeToServerPush(char* emsg, size_t emsgsize) {
  // could be ClientPull, ClientPullPreFetch or ServerPush
  // we use serverpush for timing : see vicon sdk doc
  return handle_result(client_cpp.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush).Result, emsg, emsgsize); // 0:success / -1:error
}

int viconsdk_getFrame(char* emsg, size_t emsgsize) {
  // request a frame of data from server (even necessary in serverpush...)
  return handle_result(client_cpp.GetFrame().Result, emsg, emsgsize); // 0:success / -1:error
}

/*int viconsdk_getFrameNumber(unsigned int* out_nframe, char* emsg, size_t emsgsize) {
    ViconDataStreamSDK::CPP::Output_GetFrameNumber fn = client_cpp.GetFrameNumber();
    *out_nframe = fn.FrameNumber;
    return handle_result(fn.Result, emsg, emsgsize); // 0:success / -1:error
  }*/

int viconsdk_enableSegmentData(int yesno, char* emsg, size_t emsgsize) {
  if(yesno) {
    return handle_result(client_cpp.EnableSegmentData().Result, emsg, emsgsize); // 0:success / -1:error
  } else {
    return handle_result(client_cpp.DisableSegmentData().Result, emsg, emsgsize); // 0:success / -1:error
  }
}

int viconsdk_enableMarkerData(int yesno, char* emsg, size_t emsgsize) {
  if(yesno) {
    return handle_result(client_cpp.EnableMarkerData().Result, emsg, emsgsize); // 0:success / -1:error
  } else {
    return handle_result(client_cpp.DisableMarkerData().Result, emsg, emsgsize); // 0:success / -1:error
  }
}

int viconsdk_enableUnlabeledMarkerData(int yesno, char* emsg, size_t emsgsize) {
  if(yesno) {
    return handle_result(client_cpp.EnableUnlabeledMarkerData().Result, emsg, emsgsize); // 0:success / -1:error
  } else {
    return handle_result(client_cpp.DisableUnlabeledMarkerData().Result, emsg, emsgsize); // 0:success / -1:error
  }
}

int viconsdk_isSegmentDataEnabled() {
  return client_cpp.IsSegmentDataEnabled().Enabled;
}

int viconsdk_isMarkerDataEnabled() {
  return client_cpp.IsMarkerDataEnabled().Enabled;
}

int viconsdk_isUnlabeledMarkerDataEnabled() {
  return client_cpp.IsUnlabeledMarkerDataEnabled().Enabled;
}
}

/*
what's not implemented here compared to tk2:

getversion		    useless
setaxismapping		useless
getaxismapping		useless
getframenumber		doesn't seem usefull either
*/
