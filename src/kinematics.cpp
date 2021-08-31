#include <tracked_robot_tools/kinematics.h>

using namespace tracked_robot_tools;

Kinematics2D::Kinematics2D(double tracksDistance, double steeringEfficiency)
  : tracksDistance(tracksDistance), steeringEfficiency(steeringEfficiency)
{}

void Kinematics2D::tracksToTwist(double vl, double vr, double *v, double *w) const
{
  *v = (vl + vr) / 2.0;
  *w = (vr - vl) * (this->steeringEfficiency / this->tracksDistance);
}

void Kinematics2D::twistToTracks(double *vl, double *vr, double v, double w) const
{
  *vr = v + w * this->tracksDistance / (2.0 * this->steeringEfficiency);
  *vl = v - w * this->tracksDistance / (2.0 * this->steeringEfficiency);
}

double Kinematics2D::getMaxAngularSpeed(double maxTrackSpeed) const
{
  double v = 0.0, w = 0.0;
  tracksToTwist(-maxTrackSpeed, maxTrackSpeed, &v, &w);
  return w;
}

void Kinematics2D::setSteeringEfficiency(double steeringEfficiency)
{
  this->steeringEfficiency = steeringEfficiency;
}
