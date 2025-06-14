// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequentials;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralDeAlg extends SequentialCommandGroup {
  /** Creates a new CoralDeAlg. */
  public CoralDeAlg(Elevator s_elevator, Manipulator s_manipulator, Wrist s_wrist, Boolean isHighAlgae) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    if(isHighAlgae){
      addCommands(new InstantCommand(() -> s_wrist.setWristPos(Constants.Wrist.algaeDegrees)));
      addCommands(new InstantCommand( () -> s_elevator.setElevatorPos(1.06)));
    } else {
      addCommands(new InstantCommand(() -> s_wrist.setWristPos(Constants.Wrist.algaeDegrees)));
      addCommands(new InstantCommand( () -> s_elevator.setElevatorPos(0.54)));
    }
  }
}
 