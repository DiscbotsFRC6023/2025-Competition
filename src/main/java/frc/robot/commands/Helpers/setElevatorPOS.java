// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Helpers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class setElevatorPOS extends Command {
  /** Creates a new setElevatorPOS. */
  public double targetPos = 0.0;
  Elevator s_elevator;
  boolean finished = false;

  public setElevatorPOS(Elevator s_elevator, double tar) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_elevator = s_elevator;
    targetPos = tar;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(s_elevator.getLiftPosition() == targetPos){
      finished = true;
    } else {
      s_elevator.goToSetpoint(targetPos);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
