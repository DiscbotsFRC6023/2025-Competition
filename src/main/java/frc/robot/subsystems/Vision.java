// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Centimeters;

import java.util.ArrayList;
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
    new Translation3d(Centimeters.of(-19.05), Centimeters.of(30.48), Centimeters.of(15.95)), 
    new Rotation3d(Math.toRadians(0.0), Math.toRadians(15.0), Math.toRadians(-22.5))
  );

  private Transform3d rightToRobot = new Transform3d(
    new Translation3d(Centimeters.of(19.05), Centimeters.of(30.48), Centimeters.of(15.95)), 
    new Rotation3d(Math.toRadians(0.0), Math.toRadians(15.0), Math.toRadians(22.5))
  );

  private PhotonPipelineResult latestLeftResult;
  private PhotonPipelineResult latestRightResult;
  private Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.8, 0.8, 0.8); // X, Y, and θ (radians)

  // Camera Pose estimators:
  private PhotonPoseEstimator leftPoseEstimator;
  private PhotonPoseEstimator rightPoseEstimator;
  public double poseTimestampOne = 0.00;
  public double poseTimestampTwo = 0.00;

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

  public boolean leftHasTarget(){
    return leftArducam.getLatestResult().hasTargets();
  }

  public boolean rightHasTarget(){
    return rightArducam.getLatestResult().hasTargets();
  }

  public PhotonTrackedTarget getLeftTarget(){
    if(leftHasTarget()){
      return leftArducam.getLatestResult().getBestTarget();
    }
    return new PhotonTrackedTarget();
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

  public Matrix<N3, N1> getstdDevs(){
    return visionStdDevs;
  }

  public ArrayList<Optional<Pose2d>> getEstimatedGlobalPose(Pose2d previousEstimatedRobotPose){
    PhotonPipelineResult leftResult = leftArducam.getLatestResult();
    PhotonPipelineResult rightResult = rightArducam.getLatestResult();

    ArrayList<Optional<Pose2d>> resultList = new ArrayList<Optional<Pose2d>>();

    // Setting reference pose
    leftPoseEstimator.setReferencePose(previousEstimatedRobotPose);
    rightPoseEstimator.setReferencePose(previousEstimatedRobotPose);

    // Get pose estimates of all cameras
    Optional<EstimatedRobotPose> pose1 = leftPoseEstimator.update(leftResult);
    Optional<EstimatedRobotPose> pose2 = rightPoseEstimator.update(rightResult);

    if(pose1.isPresent() && pose2.isPresent()){ // Using both
      poseTimestampOne = pose1.get().timestampSeconds;
      poseTimestampTwo = pose2.get().timestampSeconds;
      resultList.add(Optional.of(pose1.get().estimatedPose.toPose2d()));
      resultList.add(Optional.of(pose2.get().estimatedPose.toPose2d()));

      return resultList;

    } else if(pose1.isPresent()){ // Just pose 1
      
      poseTimestampOne = pose1.get().timestampSeconds;
      resultList.add(Optional.of(pose1.get().estimatedPose.toPose2d()));

      return resultList;
    
    } else if(pose2.isPresent()){ // Just pose 2
      poseTimestampOne = pose2.get().timestampSeconds;
      resultList.add(Optional.of(pose2.get().estimatedPose.toPose2d()));
      
      return resultList;

    }

    return resultList;
  }
}