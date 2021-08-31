#pragma once

namespace tracked_robot_tools
{

/** \brief 2D Motion model. */
class Kinematics2D
{

protected:
  double tracksDistance; /**< \brief The distance between the centers of the two tracks [m]. */
  double steeringEfficiency; /**< \brief Steering efficiency coefficient (range 0.0 - 1.0). */

public:
  /**
   * \brief Create the kinematics model with the given vehicle width and steering efficiency.
   * \param tracksDistance Distance between centers of both tracks [m].
   * \param steeringEfficiency Steering efficiency coefficient (0.0 - 1.0). Pass 1.0 if you
   *                           want to convert commanded cmd_vel to a tracks velocity command.
   *                           Pass the best guess of steering efficiency if you want to utilize
   *                           this class to estimate odometry from measured tracks velocity.
   */
  Kinematics2D(double tracksDistance, double steeringEfficiency);

  /**
   * \brief Set the steering efficiency used by the kinematics model.
   * \param steeringEfficiency The steering efficiency to set.
   */
  void setSteeringEfficiency(double steeringEfficiency);

  /**
   * \brief Compute linear and angular velocity based on tracks velocity
   * taking steering efficiency into account.
   *
   * \param [in] vl Left track velocity [m/s].
   * \param [in] vr Right track velocity [m/s].
   * \param v [out] Forward velocity [m/s].
   * \param w [out] Angular velocity [rad/s].
   */
  void tracksToTwist(double vl, double vr, double *v, double *w) const;

  /**
   * \brief computes tracks velocity based on linear and angular velocity
   * taking steering efficiency into account.
   *
   * \param v [in] Forward velocity [m/s].
   * \param w [in] Angular velocity [rad/s].
   * \param [out] vl Left track velocity [m/s].
   * \param [out] vr Right track velocity [m/s].
   */
  void twistToTracks(double *vl, double *vr, double v, double w) const;

  /**
   * \brief Return the maximum angular speed achievable with the given maximum track speed.
   * \param maxTrackSpeed Maximum track speed [m/s].
   * \return The maximum angular speed.
   */
  double getMaxAngularSpeed(double maxTrackSpeed) const;
};

}