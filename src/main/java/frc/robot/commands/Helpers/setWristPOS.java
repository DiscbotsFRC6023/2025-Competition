// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Helpers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class setWristPOS extends Command {
  /** Creates a new setElevatorPOS. */
  public double targetPos = 0.0;
  Wrist s_wrist;
  boolean finished = false;

  public setWristPOS(Wrist s_wrist, double tar) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_wrist = s_wrist;
    targetPos = tar;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(s_wrist.getWristPosInDegrees() == targetPos){
      finished = true;
    } else {
      s_wrist.setWristPos(targetPos);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_wrist.setWristPos(targetPos);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
