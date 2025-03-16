// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Helpers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Score extends Command {
  /** Creates a new Score. */
  Manipulator s_manipulator;
  Timer time = new Timer();
  boolean finished = false;

  public Score(Manipulator s_manipulator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_manipulator = s_manipulator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_manipulator.intakeCoral(0.20);
    time.reset();
    time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(time.get() >= 2.0){
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_manipulator.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
