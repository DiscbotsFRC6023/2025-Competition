// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private final PhotonCamera leftArducam;
  private final PhotonCamera rightArducam;

  private Transform3d leftToRobot = new Transform3d(
    new Translation3d(-19.05, 30.48, 15.95), 
    new Rotation3d(Math.toRadians(0.0), Math.toRadians(15.0), Math.toRadians(-22.5))  //FIXME switch to radians
  );

  private Transform3d rightToRobot = new Transform3d(
    new Translation3d(19.05, 30.48, 15.95), 
    new Rotation3d(Math.toRadians(0.0), Math.toRadians(15.0), Math.toRadians(-22.5))  //FIXME switch to radians
  );

  private PhotonPipelineResult latestLeftResult;
  private PhotonPipelineResult latestRightResult;

  // Camera Pose estimators:
  private PhotonPoseEstimator leftPoseEstimator;
  private PhotonPoseEstimator rightPoseEstimator;
  public double poseTimestamp = 0.00;

  public boolean hasTarget = false;

  public Vision() {
    leftArducam = new PhotonCamera("Left_Arducam");
    rightArducam = new PhotonCamera("Right_Arducam");

    leftPoseEstimator = new PhotonPoseEstimator(
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape), 
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
      leftToRobot
    );

    rightPoseEstimator = new PhotonPoseEstimator(
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape), 
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
      rightToRobot
    );

    // Setup Network Tables:
    NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    NetworkTable poseTable = ntInstance.getTable("PoseData");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    hasTarget = this.hasTarget();
  }

  public PhotonTrackedTarget getBestTarget(){
    if(hasTarget){
      return latestLeftResult.getBestTarget();
    }
    return new PhotonTrackedTarget();
  }

  public boolean leftHasTarget(){
    return leftArducam.getLatestResult().hasTargets();
  }

  public boolean rightHasTarget(){
    return rightArducam.getLatestResult().hasTargets();
  }

  public double getLeftX(int targetID){
        var latestResult = leftArducam.getLatestResult();
        if (latestResult.hasTargets()) {
            for (int i = 0; i<latestResult.getTargets().size(); i++){
                    if (latestResult.getTargets().get(i).getFiducialId() == targetID) {
                        return latestResult.getTargets().get(i).getBestCameraToTarget().getX();
                    }
                   }
                   return 0;
            }
        return 0;
  }

  public double getRightX(int targetID){
    var latestResult = rightArducam.getLatestResult();
    if (latestResult.hasTargets()) {
        for (int i = 0; i<latestResult.getTargets().size(); i++){
                if (latestResult.getTargets().get(i).getFiducialId() == targetID) {
                    return latestResult.getTargets().get(i).getBestCameraToTarget().getX();
                }
               }
               return 0;
        }
    return 0;
}

public double getLeftY(int targetID){
  var latestResult = leftArducam.getLatestResult();
  if (latestResult.hasTargets()) {
      for (int i = 0; i<latestResult.getTargets().size(); i++){
              if (latestResult.getTargets().get(i).getFiducialId() == targetID) {
                  return latestResult.getTargets().get(i).getBestCameraToTarget().getY();
              }
             }
             return 0;
      }
  return 0;
}

public double getRightY(int targetID){
  var latestResult = rightArducam.getLatestResult();
  if (latestResult.hasTargets()) {
      for (int i = 0; i<latestResult.getTargets().size(); i++){
              if (latestResult.getTargets().get(i).getFiducialId() == targetID) {
                  return latestResult.getTargets().get(i).getBestCameraToTarget().getY();
              }
             }
             return 0;
      }
  return 0;
}

  public boolean hasID(int targetID){
    if (
      latestLeftResult.getBestTarget().getFiducialId() == targetID ||
      latestRightResult.getBestTarget().getFiducialId() == targetID
    ) {
      return true;
    }
    return false;
  }

  public boolean hasTarget(){
    return leftHasTarget() || rightHasTarget();
  }

  public Optional<Pose2d> getEstimatedGlobalPose(Pose2d previousEstimatedRobotPose){
    PhotonPipelineResult leftResult = leftArducam.getLatestResult();
    PhotonPipelineResult rightResult = rightArducam.getLatestResult();
    
    // Setting reference pose
    leftPoseEstimator.setReferencePose(previousEstimatedRobotPose);
    rightPoseEstimator.setReferencePose(previousEstimatedRobotPose);

    // Get pose estimates of all cameras
    Optional<EstimatedRobotPose> pose1 = leftPoseEstimator.update(leftResult);
    Optional<EstimatedRobotPose> pose2 = leftPoseEstimator.update(rightResult);

    if(pose1.isPresent() && pose2.isPresent()){ // Using both by avg
      Pose2d avgPose = new Pose2d(
        pose1.get().estimatedPose.toPose2d().getTranslation()
        .plus(pose2.get().estimatedPose.toPose2d().getTranslation()).div(2),
        pose1.get().estimatedPose.toPose2d().getRotation()
      );
      poseTimestamp = pose1.get().timestampSeconds;
      //return Optional.of(avgPose);
    } else if(pose1.isPresent()){ // Just pose 1
      poseTimestamp = pose1.get().timestampSeconds;
      return Optional.of(pose1.get().estimatedPose.toPose2d());
    } else if(pose2.isPresent()){ // Just pose 2
      poseTimestamp = pose2.get().timestampSeconds;
      return Optional.of(pose2.get().estimatedPose.toPose2d());
    }

    return Optional.empty();
  }
}