// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// BROKEN AS OF THE 2025 SEASON VERSIONS

package frc.robot.RobotVision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotVision.VisionConstants.PhotonConstants;
import frc.robot.RobotVision.VisionConstants.PositionConstants;

// This class is responsible for processing the results
// of the vision data streamed over via from the Pi

public class VisionSubsystem extends SubsystemBase {

  // THIS CLASS IS IN ITS INFANCY, AND IS SUBJECT TO LARGE CHANGES

  // Create camera objects for each camera on the robot.
  // The camera name should be the name of the Network Table
  // that contains the camera stream.
  public PhotonCamera camera1 = new PhotonCamera(PhotonConstants.cameraName1);

  // Declare a variable to refrence a pose estimator
  private Pose2d robotPose;

  // Create a photon pose estimator for april tag, field pose estimation
  private PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(VisionConstants.PositionConstants.tagPositions,
  PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
  PositionConstants.robotToCam);

  // Get a robot pose source from the instantiator
  public VisionSubsystem(Pose2d robotPose) { this.robotPose = robotPose; }


  /* ----------------------------------------------------
   * Fundamental methods for basic camera functions
   * -------------------------------------------------- */

  // Returns the latest information from photonvision
  public PhotonPipelineResult getLatest(PhotonCamera photonCamera) {
    List<PhotonPipelineResult> result = photonCamera.getAllUnreadResults();
    return result.isEmpty() ? null : result.get(0);
  }

  // Returns whether photon sees one or more apriltags
  public boolean hasTarget(PhotonCamera photonCamera) {
    return getLatest(photonCamera).hasTargets();
  }

  // Returns a list of targets that photon sees
  public List<PhotonTrackedTarget> getVisibleTargets(PhotonCamera photonCamera) {
    return getLatest(photonCamera).getTargets();
  }

  // Returns the best target in view of the photon camera
  public PhotonTrackedTarget getBestVisibleTarget(PhotonCamera photonCamera) {
    return getLatest(photonCamera).getBestTarget();
  }


  /* ----------------------------------------------------
   * Utilities for common vision processing techniques
   * -------------------------------------------------- */


  // Returns yaw difference to a given pose on the field
  public Rotation2d getYawToPose(Pose2d currentPose, Pose2d targetPose) {
    return PhotonUtils.getYawToPose(currentPose, targetPose);
  }

  // Returns yaw difference to a given tag on the field
  public Rotation2d getYawToTag(Pose2d currentPose, int tagId) {
    Pose2d tagPose; // The tag poase shouldnt be null, but in the case that it is, well pass the problem onto the caller
    try { tagPose = PositionConstants.tagPositions.getTagPose(tagId).get().toPose2d(); } catch (Exception e) { return null; }
    return PhotonUtils.getYawToPose(currentPose, tagPose);
  }

  // Will return the distance in meters to a given pose on the field
  public double getDistanceToPose(Pose2d currentPose, Pose2d targetPose) {
    double distanceToTargetPose = PhotonUtils.getDistanceToPose(currentPose, targetPose);
    return distanceToTargetPose;
  }

  // Will return the distance in meters from the specified tag
  public double getDistanceToTag(Pose2d currentPose, int tagId) {
    Pose2d tagPose; // The tag poase shouldnt be null, but in the case that it is, well pass the problem onto the caller via a -1
    try { tagPose = PositionConstants.tagPositions.getTagPose(tagId).get().toPose2d(); } catch (Exception e) { return -1; }
    double result = PhotonUtils.getDistanceToPose(currentPose, tagPose);
    return result;
  }

  // Grab specific target based off of ID
  public PhotonTrackedTarget getSpecifiedTarget(PhotonCamera photonCamera, int apriltagId) {
      List<PhotonTrackedTarget> tags = getVisibleTargets(photonCamera);
      for (PhotonTrackedTarget tag : tags) {
        if (tag.getFiducialId() == apriltagId) {
          return tag;
        }
      }
      return null;
  }

  // Get updated estimate of the global robot pose based off of april tags.
  public Pose2d getEstimatedGlobalPose() {
    poseEstimator.setReferencePose(robotPose);
    try {
      Optional<EstimatedRobotPose> estimate = poseEstimator.update(getLatest(camera1));
      if (estimate.isPresent()) {
        Pose3d estimatedRobotPose = estimate.get().estimatedPose;
        return estimatedRobotPose.toPose2d();
      } else {
        return null;
      }
    } catch (Exception e) {
      return null;
    }
  }

  // A convenience function to compile the data of the best
  // tag in view into a single array.
  public double[] getBestTagData(PhotonCamera photonCamera) {
    double[] data = new double[6];
    if (photonCamera.getAllUnreadResults().get(0).hasTargets()) {
      data[0] = getBestVisibleTarget(photonCamera).getFiducialId();
      data[1] = getBestVisibleTarget(photonCamera).getPitch();
      data[2] = getBestVisibleTarget(photonCamera).getYaw();
      data[3] = getBestVisibleTarget(photonCamera).getSkew();
      data[4] = getBestVisibleTarget(photonCamera).getArea();
    }
    data[5] = photonCamera.getAllUnreadResults().get(0).hasTargets() ? 1:0;
    return data;
  }

  // A convenience function to compile the data of the specified
  // tag in view into a single array.
  public double[] getSpecifiedTagData(PhotonCamera photonCamera, int tagId) {
    double[] data = new double[6];
    PhotonTrackedTarget target = getSpecifiedTarget(photonCamera, tagId);
    if (photonCamera.getAllUnreadResults().get(0).hasTargets() & target != null) {
      data[0] = getSpecifiedTarget(photonCamera, tagId).getFiducialId();
      data[1] = getSpecifiedTarget(photonCamera, tagId).getPitch();
      data[2] = getSpecifiedTarget(photonCamera, tagId).getYaw();
      data[3] = getSpecifiedTarget(photonCamera, tagId).getSkew();
      data[4] = getSpecifiedTarget(photonCamera, tagId).getArea();
    }
    data[5] = photonCamera.getAllUnreadResults().get(0).hasTargets() ? 1:0;
    return data;
  }


  @Override
  public void periodic() {
    //getEstimatedGlobalPose();
  }
}
