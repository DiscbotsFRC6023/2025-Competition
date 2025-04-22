// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Helpers;

import java.util.List;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPose extends Command {
  /** Creates a new DriveToPose. */
  private CommandSwerveDrivetrain s_swerve;
  private Pose2d targetPose;
  private PathPlannerPath generatedPath;
  private double goalEndHeading;
  private boolean finished = false;

  public DriveToPose(Pose2d targetPose, double endStateHeading, CommandSwerveDrivetrain s_swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_swerve = s_swerve;
    this.targetPose = targetPose;
    goalEndHeading = endStateHeading;
    addRequirements(s_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      targetPose
    );

    PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
    // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

    // Create the path using the waypoints created above
    generatedPath = new PathPlannerPath(
            waypoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState(0.0, Rotation2d.fromDegrees(goalEndHeading)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Command swerveCommand = s_swerve.followPathCommand(generatedPath);

    if(swerveCommand.isFinished()){
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
