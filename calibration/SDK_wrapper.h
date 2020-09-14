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


// wrapper for Vicon SDK : access the client methods through C functions
// an instance of the client cpp object is created in SDK_wrapper.cpp

#ifndef H_SDK_WRAPPER
#define H_SDK_WRAPPER

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>

int viconsdk_getSubjectCount(unsigned int* out_count, char* emsg, size_t emsgsize);
int viconsdk_getMarkerCount(const char* SubjectName, unsigned int* out_count, char* emsg, size_t emsgsize);
int viconsdk_getUnlabeledMarkerCount(unsigned int* out_count, char* emsg, size_t emsgsize);
int viconsdk_getSubjectName(const unsigned int SubjectIndex, char* out_name, size_t name_size, char* emsg, size_t emsgsize);
int viconsdk_getSubjectRootSegmentName(const char* SubjectName, char* out_name, size_t rsname_size, char* emsg, size_t emsgsize);
int viconsdk_getMarkerName(const char* SubjectName, const unsigned int MarkerIndex, char* out_name, size_t mname_size, char* emsg, size_t emsgsize);
int viconsdk_getSegmentPos(const char* SubjectName, const char* SegmentName, double* out_x, double* out_y, double* out_z, int* out_occluded, char* emsg, size_t emsgsize);
int viconsdk_getSegmentQuat(const char* SubjectName, const char* SegmentName, double* out_qw, double* out_qx, double* out_qy, double* out_qz, int* out_occluded, char* emsg, size_t emsgsize);
int viconsdk_getMarkerPos(const char* SubjectName, const char* MarkerName, double* out_x, double* out_y, double* out_z, int* out_occluded, char* emsg, size_t emsgsize);
int viconsdk_getUnlabeledMarkerPos(const unsigned int MarkerIndex, double* out_x, double* out_y, double* out_z, char* emsg, size_t emsgsize);
int viconsdk_connect(const char* host_extended, char* emsg, size_t emsgsize);
int isConnected();
int viconsdk_disconnect(char* emsg, size_t emsgsize);
int viconsdk_setStreamModeToServerPush(char* emsg, size_t emsgsize);
int viconsdk_getFrame(char* emsg, size_t emsgsize);
int viconsdk_enableSegmentData(int yesno, char* emsg, size_t emsgsize);
int viconsdk_enableMarkerData(int yesno, char* emsg, size_t emsgsize);
int viconsdk_enableUnlabeledMarkerData(int yesno, char* emsg, size_t emsgsize);
int viconsdk_isSegmentDataEnabled();
int viconsdk_isMarkerDataEnabled();
int viconsdk_isUnlabeledMarkerDataEnabled();

// structures to store tracker data: (taken from optitrack component)
struct tracker_anonmset {
  uint32_t n;			/* number of markers */
  struct __attribute__ ((__packed__)) {
    float x, y, z;
  } *pos;			/* markers position */
};

struct tracker_msetdescr {
  char *name;			/* model name */
  char safename[64];		/* name without special chars */

  uint32_t nmnames;		/* number of marker names */
  char **lmname;			/* marker names */
};

struct tracker_mset {
  const struct tracker_msetdescr *descr;
  const char *name;		/* model name */
  struct tracker_anonmset mset;	/* markers */
};

struct tracker_bodydescr {
  char *name;			/* body name */
  char *rsname;			/* root segment name */
  char safename[64];		/* name without special chars */
  char rssafename[64];		/* rsname without special chars */
};

struct tracker_body {
  const struct tracker_bodydescr *descr;
  struct __attribute__ ((__packed__)) {
    float x, y, z, qx, qy, qz, qw;
  } pose;			/* body pose */

  struct tracker_anonmset mset;	/* associated marker set */
};

struct tracker_descr {
  uint32_t nmsets;	 	 /* number of marker sets descriptions */
  struct tracker_msetdescr *mset;/* marker sets descriptions */

  uint32_t nbodies;	 	 /* number of bodies descriptions */
  struct tracker_bodydescr *body;/* bodies descriptions */
};

struct tracker_data {
  //timestamp ? how to get vicon's one ?
  struct tracker_anonmset aset;  /* anonymous markers set */

  uint32_t nmsets;		/* number of markers sets */
  struct tracker_mset *mset;	/* markers sets */

  uint32_t nbodies;		/* number of rigid bodies */
  struct tracker_body *body;	/* rigid bodies */
};


#ifdef __cplusplus
} // extern "C"
#endif

#endif /* H_SDK_WRAPPER */

