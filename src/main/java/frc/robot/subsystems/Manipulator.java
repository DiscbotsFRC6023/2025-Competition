// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotStates;

public class Manipulator extends SubsystemBase {
  /** Creates a new Manipulator. */
  private final TalonFX manipulatorMotor;
  private DigitalInput coralSensor;
  private RobotStates sharedStates;

  public Manipulator(RobotStates sharedStates){
    manipulatorMotor = new TalonFX(Constants.Manipulator.MAN_CANID);
    coralSensor = new DigitalInput(Constants.Manipulator.CORAL_SENSOR_PORT);
    manipulatorMotor.getConfigurator().apply(Constants.Manipulator.MANIPULATOR_CONFIG);
    this.sharedStates = sharedStates;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Coral Present:", this.getCoralSensor());
    sharedStates.hasCoral = this.getCoralSensor();
  }

  public boolean getCoralSensor(){
    return !coralSensor.get();
  }

  public double getIntakeRotations(){
    return manipulatorMotor.getPosition().getValueAsDouble(); 
  }

  public void resetEncoder(){
    manipulatorMotor.setPosition(0.0);
  }

  public void intakeAlgae(){
    manipulatorMotor.set(Constants.Manipulator.ALGAE_INTAKE_SPEED);
  }

  public void throwup(){
    manipulatorMotor.set(0.15);
  }

  public void intakeAlgae(double speed){
    manipulatorMotor.set(speed);
  }

  public void intakeCoral(){
    manipulatorMotor.set(-Constants.Manipulator.CORAL_INTAKE_SPEED);
  }

  public void intakeCoral(double speed){
    manipulatorMotor.set(-speed);
  }

  public void stopAll(){
    manipulatorMotor.stopMotor();
  }

  public void holdBall(){
    this.intakeAlgae(0.1);
  }
}
