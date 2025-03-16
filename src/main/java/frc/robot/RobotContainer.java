// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Helpers.IntakeCoral;
import frc.robot.commands.Helpers.Score;
import frc.robot.commands.Sequentials.*;
import frc.robot.subsystems.*;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.95).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity  //0.75
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final XboxController controller = new XboxController(1);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public SendableChooser<Command> autonChooser;


    private Manipulator s_manipulator = new Manipulator();
    private Elevator s_elevator = new Elevator();
    private Wrist s_wrist = new Wrist();
    private JoystickButton a = new JoystickButton(controller, XboxController.Button.kA.value);
    private JoystickButton x = new JoystickButton(controller, XboxController.Button.kX.value);
    private JoystickButton b = new JoystickButton(controller, XboxController.Button.kB.value);
    private JoystickButton y = new JoystickButton(controller, XboxController.Button.kY.value);
    private JoystickButton home = new JoystickButton(controller, XboxController.Button.kRightStick.value);
    private JoystickButton algaeHome = new JoystickButton(controller, XboxController.Button.kLeftStick.value);
    private JoystickButton intake = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
    private JoystickButton score = new JoystickButton(controller, XboxController.Button.kRightBumper.value);

    public RobotContainer() {
        CameraServer.startAutomaticCapture();
        configureBindings();

        NamedCommands.registerCommand("Home", new Home(s_elevator, s_manipulator, s_wrist));
        NamedCommands.registerCommand("L2", new L2(s_elevator, s_manipulator, s_wrist));
        NamedCommands.registerCommand("L3", new MidL3Combo(s_elevator, s_manipulator, s_wrist));
        NamedCommands.registerCommand("Score", new Score(s_manipulator));

        autonChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser:", autonChooser);
    }

    private void configureBindings() {
        a.onTrue(new InstantCommand(() -> {
            if (s_manipulator.getCoralSensor()) {
                //new RunCommand(() -> s_manipulator.intakeCoral(0.1), s_manipulator).withTimeout(1.0).andThen(new InstantCommand(() -> s_manipulator.stopAll(), s_manipulator)).schedule();
            } else {
                new Processor(s_elevator, s_manipulator, s_wrist).schedule();
            }
          }));
      
          x.onTrue(new InstantCommand(() -> {
            if (s_manipulator.getCoralSensor()) {
                new L2(s_elevator, s_manipulator, s_wrist).schedule();
            } else {
                new L1Algae(s_elevator, s_manipulator, s_wrist).schedule();
            }
          }));
      
          y.onTrue(new InstantCommand(() -> {
            if (s_manipulator.getCoralSensor()) {
                new L3(s_elevator, s_manipulator, s_wrist).schedule();
            } else {
                new L2Algae(s_elevator, s_manipulator, s_wrist).schedule();
            }
          }));
      
          b.onTrue(new InstantCommand(() -> {
            if (s_manipulator.getCoralSensor()) {
                new L4(s_elevator, s_manipulator, s_wrist).schedule();
            } else {
                new Barge(s_elevator, s_manipulator, s_wrist).schedule();
            }
          }));
      
          intake.onTrue(new IntakeCoral(s_manipulator).andThen(new RunCommand(() -> s_manipulator.intakeCoral(0.1), s_manipulator).withTimeout(0.2).andThen(new InstantCommand(() -> s_manipulator.stopAll()))));
      
          score.onTrue(new InstantCommand(() -> {
            if (s_manipulator.getCoralSensor()) {
              new RunCommand(() -> s_manipulator.intakeCoral(0.4), s_manipulator).schedule();
            } else {
              new RunCommand(() -> s_manipulator.intakeCoral(1.0), s_manipulator).schedule();
            }
          }));
          score.onFalse(new InstantCommand(() -> s_manipulator.stopAll(), s_manipulator));
      
          home.onTrue(new Home(s_elevator, s_manipulator, s_wrist));
          algaeHome.onTrue(new AlgaeHome(s_elevator, s_manipulator, s_wrist));

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.rightBumper().whileTrue(
          drivetrain.applyRequest(() ->
            drive.withVelocityX(-joystick.getLeftY() * (MaxSpeed / 3))
              .withVelocityY(-joystick.getLeftX() * (MaxSpeed / 3))
              .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
          );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
      try{
        return autonChooser.getSelected();
      } catch(AutoBuilderException abe){
        return new PrintCommand("PATRICK MAHOMES: " + "\n" + abe.getMessage());
      }
    }
}
