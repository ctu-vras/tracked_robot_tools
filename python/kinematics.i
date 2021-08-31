%module kinematics

%feature("autodoc", "1");

%{
#include "tracked_robot_tools/kinematics.h"
%}

%include "typemaps.i"
%apply double *OUTPUT { double *v, double *w};  // Kinematics2D::tracksToTwist
%apply double *OUTPUT { double *vl, double *vr};  // Kinematics2D::twistToTracks

%include "tracked_robot_tools/kinematics.h"
