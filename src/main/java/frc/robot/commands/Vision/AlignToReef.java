// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import java.util.ArrayList;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.Helpers.DriveToPose;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToReef extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private CommandSwerveDrivetrain s_swerve;
  private Vision s_vision;
  private int tagID = -1;
  private boolean isRedAlliance;
  private ArrayList<Integer> tagIDees;

  private double X_REEF_ALIGNMENT_P = 0.1;
  private double Y_REEF_ALIGNMENT_P = 0.1;
  private double ROT_REEF_ALIGNMENT_P = 0.1;
  private double LEFT_Y_SETPOINT_REEF_ALIGNMENT = -0.14;
  private double RIGHT_Y_SETPOINT_REEF_ALIGNMENT = 0.0;

  public AlignToReef(boolean isRightScore, CommandSwerveDrivetrain s_swerve, Vision s_vision) {
    this.isRightScore = isRightScore;
    this.s_swerve = s_swerve;
    this.s_vision = s_vision;
    addRequirements(s_swerve, s_vision);

    xController = new PIDController(X_REEF_ALIGNMENT_P, 0, 0);
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
  }

  @Override
  public void execute() { // IMPLEMENT THIS
    if(s_vision.hasTarget()){
      PhotonTrackedTarget target = s_vision.getBestTarget();
      if(tagIDees.contains(target.getFiducialId())){
        // Calculate pose

        // Start commanding
        new DriveToPose(s_swerve, s_vision).schedule();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    s_swerve.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
  }

  private double getAngle(int id){
    if (id == 7 ||id == 18) {
      return 0;
    }else if (id == 8 || id == 17) {
      return 60;
    }else if (id == 6 || id == 19) {
      return -60;
    }else if (id == 11 || id == 20) {
      return -120;
    }else if (id == 10 || id == 21) {
      return 180;
    }else if (id == 9 || id == 22) {
      return 120;  
    }else {
      return s_swerve.getPose().getRotation().getDegrees();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}