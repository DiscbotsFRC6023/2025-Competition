// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import java.util.ArrayList;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAlignToReef extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private CommandSwerveDrivetrain s_swerve;
  private Vision s_vision;
  private int tagID = -1;
  private boolean isRedAlliance;
  private ArrayList<Integer> tagIDees;
  private double manualOffset;
  private PhotonTrackedTarget initialTarget;

  private double X_REEF_ALIGNMENT_P = 2.5;
  private double Y_REEF_ALIGNMENT_P = 2.0;
  private double ROT_REEF_ALIGNMENT_P = 1.0;
  private double LEFT_Y_SETPOINT_REEF_ALIGNMENT = -0.32; //-0.3
  private double RIGHT_Y_SETPOINT_REEF_ALIGNMENT = 0.04;

  public AutoAlignToReef(boolean isRightScore, CommandSwerveDrivetrain s_swerve, Vision s_vision) {
    this.isRightScore = isRightScore;
    this.s_swerve = s_swerve;
    this.s_vision = s_vision;
    addRequirements(s_swerve, s_vision);

    xController = new PIDController(X_REEF_ALIGNMENT_P, 0, 0.001);
    yController = new PIDController(Y_REEF_ALIGNMENT_P, 0, 0);
    rotController = new PIDController(ROT_REEF_ALIGNMENT_P, 0, 0);
  }

  public AutoAlignToReef(double manualOffset, CommandSwerveDrivetrain s_swerve, Vision s_vision) {
    this.s_swerve = s_swerve;
    this.s_vision = s_vision;
    this.manualOffset = manualOffset;
    addRequirements(s_swerve, s_vision);

    xController = new PIDController(X_REEF_ALIGNMENT_P, 0, 0.001);
    yController = new PIDController(Y_REEF_ALIGNMENT_P, 0, 0);
    rotController = new PIDController(ROT_REEF_ALIGNMENT_P, 0, 0);
  }

  @Override
  public void initialize() {
    tagIDees = new ArrayList<Integer>();
    isRedAlliance = DriverStation.getAlliance().get() == Alliance.Red;

    if(isRedAlliance){
      tagIDees.add(6);
      tagIDees.add(7);
      tagIDees.add(8);
      tagIDees.add(9);
      tagIDees.add(10);
      tagIDees.add(11);
    } else {
      tagIDees.add(17);
      tagIDees.add(18);
      tagIDees.add(19);
      tagIDees.add(20);
      tagIDees.add(21);
      tagIDees.add(22);
    }

    if(s_vision.leftHasTarget() && tagIDees.contains(s_vision.getLeftTarget().fiducialId)){
      initialTarget = s_vision.getLeftTarget();
      tagID = initialTarget.fiducialId;
    }

    if(isRightScore){
      yController.setSetpoint(RIGHT_Y_SETPOINT_REEF_ALIGNMENT);
    } else if(!isRightScore){
      yController.setSetpoint(LEFT_Y_SETPOINT_REEF_ALIGNMENT);
    } else {
      yController.setSetpoint(manualOffset);
    }

    xController.setSetpoint(0.32);
    xController.setTolerance(0.5);

    yController.setTolerance(0.2);

    // Not working
    rotController.setSetpoint(-151); //2.57
    rotController.setTolerance(0.1);

    SmartDashboard.putNumber("INIT TAG SETPOINT", initialTarget.getBestCameraToTarget().getRotation().toRotation2d().getDegrees());
  }

  @Override
  public void execute() { // IMPLEMENT THIS
    if(s_vision.leftHasTarget()){
      PhotonTrackedTarget target = s_vision.getLeftTarget();
      if(tagID == target.fiducialId){
        System.out.println("SEEING TAG " + target.getFiducialId());
        Transform3d tagInput = target.getBestCameraToTarget();
        SmartDashboard.putNumber("TAG VAL", tagInput.getRotation().toRotation2d().getDegrees());

        double rotOutput = rotController.calculate(tagInput.getRotation().getAngle());
        double xOutput = xController.calculate(tagInput.getX());
        double yOutput = yController.calculate(tagInput.getY());

        s_swerve.driveRobotRelative(new ChassisSpeeds(0, -yOutput, 0));
      }
    } else {
      System.out.println("NO TARGET");
      s_swerve.driveRobotRelative(new ChassisSpeeds());
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return yController.atSetpoint();
  }
}