// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  private final SparkMax elevatorMotorOne;
  private final SparkMax elevatorMotorTwo;
  private RelativeEncoder encoder;
  private DigitalInput elevatorHomeSwitch;
  private PIDController elevatorController;
  private ElevatorFeedforward feedforward;
  private double pidOutput = 0.0;
  private double motorOutput = 0.0;
  public double ffOutput = 0.0;


  public Elevator() {
    elevatorMotorOne = new SparkMax(Constants.Elevator.ELEVATOR_ONE_CANID, MotorType.kBrushless);
    elevatorMotorTwo = new SparkMax(Constants.Elevator.ELEVATOR_TWO_CANID, MotorType.kBrushless);

    elevatorHomeSwitch = new DigitalInput(Constants.Elevator.ELEVATOR_HOMESWITCH_PORT);
    encoder = elevatorMotorOne.getEncoder();
    elevatorController = new PIDController(Constants.Elevator.kP, Constants.Elevator.kI, Constants.Elevator.kD);
    feedforward = new ElevatorFeedforward(Constants.Elevator.kS, Constants.Elevator.kG, Constants.Elevator.kV);

    elevatorMotorOne.configure(
      Constants.Elevator.LEFT_CONFIG,
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );

    elevatorMotorTwo.configure(
      Constants.Elevator.RIGHT_CONFIG, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Elevator Homed:", this.getHomeSwitch());
    SmartDashboard.putNumber("Elevator ENC:", this.getLiftPosition());

    if(getHomeSwitch()){
      encoder.setPosition(0.0);
    }
  }

  public boolean getHomeSwitch(){
    return !elevatorHomeSwitch.get();
  }

  public double getLiftPosition(){
    return -encoder.getPosition();
  }

  public double getVelocity(){
    return encoder.getVelocity();
  }

  public void manualLift(double speed){
    elevatorMotorOne.set(speed);
  }

  public void stopAll(){
    elevatorMotorOne.stopMotor();
    elevatorMotorTwo.stopMotor();
  }

  public void goToSetpoint(double setpoint){
    SmartDashboard.putNumber("CURR SETPOINT:", setpoint);
    pidOutput = elevatorController.calculate(getLiftPosition(), setpoint);
    ffOutput = feedforward.calculate(this.getVelocity());

    // Basic Gravity Compensation:
    motorOutput = pidOutput + ffOutput;

    // Clamping:
    motorOutput = Math.min(Math.max(motorOutput, -1.0), 1.0);

    elevatorMotorOne.set(-pidOutput); // Negative so the elevator does not go the wrong way ¯\_(ツ)_/¯
  }

  public void homeElevator(){
    this.goToSetpoint(0.01);
  }
}
