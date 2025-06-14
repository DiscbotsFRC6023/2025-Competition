// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequentials;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Barge extends SequentialCommandGroup {
  /** Creates a new Barge. */
  public Barge(Elevator s_elevator, Manipulator s_manipulator, Wrist s_wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand( () -> s_elevator.setElevatorPos(Constants.Elevator.bargePosition)));
    addCommands(new InstantCommand(() -> s_wrist.setWristPos(Constants.Wrist.bargeDegrees)));

  }
}
