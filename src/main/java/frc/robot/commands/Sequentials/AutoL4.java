// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequentials;

import frc.robot.commands.Helpers.setElevatorPOS;
import frc.robot.commands.Helpers.setWristPOS;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoL4 extends SequentialCommandGroup {
  /** Creates a new L1Coral. */
  public AutoL4(Elevator s_elevator, Manipulator s_manipulator, Wrist s_wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new RunCommand(() -> s_wrist.setWristPos(25), s_wrist).withTimeout(0.8).alongWith(new WaitCommand(0.05).andThen(new setElevatorPOS(s_elevator, 1.6).withTimeout(1.5))));
    addCommands(new setWristPOS(s_wrist, 35).withTimeout(1.25)); //1.5
    addCommands(new RunCommand(() -> s_manipulator.intakeCoral(), s_manipulator).withTimeout(0.45).andThen(new InstantCommand(() -> s_manipulator.stopAll(), s_manipulator)));  
    addCommands(new Home(s_elevator, s_manipulator, s_wrist).withTimeout(0.65));
  }
}