// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequentials;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Home extends Command {
  /** Creates a new Home */
  private Elevator s_elevator;
  private Wrist s_wrist;

  public Home(Elevator s_elevator, Wrist s_wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_elevator = s_elevator;
    this.s_wrist = s_wrist;
    addRequirements(s_elevator, s_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_wrist.setWristPos(Constants.Wrist.travelDegrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(s_wrist.safeForTravel()){
      s_elevator.setElevatorPos(Constants.Elevator.homePosition);
    }
    if(s_elevator.getHomeSwitch()){
      s_wrist.homeWrist();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_wrist.getAbsWristPosInDegrees() <= Constants.Wrist.homeDegrees;
  }
}
