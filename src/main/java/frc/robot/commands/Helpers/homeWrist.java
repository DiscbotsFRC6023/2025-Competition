// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Helpers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class homeWrist extends Command {

  Wrist s_wrist;
  boolean finished = false;

  /** Creates a new IntakeCoral. */
  public homeWrist(Wrist s_wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_wrist = s_wrist;
    //addRequirements(s_manipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(s_wrist.getWristPosInDegrees() <= 17.0 && s_wrist.getWristPosInDegrees() >= 15.5){
      finished = true;
    } else {
      s_wrist.setWristPos(17.0);
    }
  }

  // Called once the command ends or is interrupted
  @Override
  public void end(boolean interrupted) {
    s_wrist.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return s_manipulator.getCoralSensor();
    return finished;
  }
}
