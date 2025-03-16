// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Helpers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HomeCoral extends Command {

  Manipulator s_manipulator;
  boolean finished = false;

  /** Creates a new IntakeCoral. */
  public HomeCoral(Manipulator s_manipulator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_manipulator = s_manipulator;
    //addRequirements(s_manipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_manipulator.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(s_manipulator.getIntakeRotations() < 2.0){
      s_manipulator.intakeCoral();      
    } else {
      s_manipulator.stopAll();
      finished = true;
    }
  }

  // Called once the command ends or is interrupted
  @Override
  public void end(boolean interrupted) {
    s_manipulator.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return s_manipulator.getCoralSensor();
    return finished;
  }
}
