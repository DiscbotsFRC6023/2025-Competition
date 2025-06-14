// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequentials;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoL4 extends SequentialCommandGroup {
  /** Creates a new AutoL4. */
  public AutoL4(Elevator s_elevator, Manipulator s_manipulator, Wrist s_wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());addCommands(new InstantCommand(() -> s_wrist.setWristPos(Constants.Wrist.scoreDegrees)));
    addCommands(new InstantCommand( () -> s_elevator.setElevatorPos(Constants.Elevator.L4)));
    addCommands(new InstantCommand(() -> s_wrist.setWristPos(Constants.Wrist.travelDegrees)));
    addCommands(new WaitCommand(5.0));
    addCommands(new InstantCommand(() -> s_wrist.setWristPos(Constants.Wrist.L4Degrees)));
    addCommands(new RunCommand(() -> s_manipulator.intakeCoral(), s_manipulator).withTimeout(0.45).andThen(new InstantCommand(() -> s_manipulator.stopAll(), s_manipulator)));  
    addCommands(new Home(s_elevator, s_wrist));
  }
}