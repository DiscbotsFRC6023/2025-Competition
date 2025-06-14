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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotStates;


public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  private final SparkMax elevatorMotorOne;
  private final SparkMax elevatorMotorTwo;
  private RelativeEncoder encoder;
  private RobotStates sharedStates;
  private DigitalInput elevatorHomeSwitch;
  private PIDController elevatorController;
  private ElevatorFeedforward feedforward;
  private double pidOutput = 0.0;
  private double motorOutput = 0.0;
  private double currentSetpoint = 0.0;
  public double ffOutput = 0.0;
  public boolean safeToTravel = false;


  public Elevator(RobotStates sharedStates) {
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

    this.sharedStates = sharedStates;
  }

  @Override
  public void periodic() {
    safeToTravel = sharedStates.elevatorSafeToTravel;
    sharedStates.elevatorHomed = this.getHomeSwitch();
    
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Elevator/Homed:", this.getHomeSwitch());
    SmartDashboard.putNumber("Elevator/Position:", this.getLiftPosition());
    SmartDashboard.putBoolean("Elevator/Safe To Travel:", safeToTravel);
    SmartDashboard.putNumber("Elevator/Current Setpoint:", currentSetpoint);
    SmartDashboard.putNumber("Elevator/Motor Commanded Power:", motorOutput);
    
    if(getHomeSwitch()){
      encoder.setPosition(0.0);
    }
    
    // Makes sure that reenabling the robot will not spring elevator back to last position
    if(DriverStation.isDisabled()){
      currentSetpoint = getLiftPosition();
    } else {
      this.driveElevator();
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
    if(safeToTravel){
      elevatorMotorOne.set(speed);
    } else {
      elevatorMotorOne.set(0.0);
    }
  }

  public void stopAll(){
    elevatorMotorOne.stopMotor();
    elevatorMotorTwo.stopMotor();
  }

  private void driveElevator(){
    pidOutput = elevatorController.calculate(getLiftPosition(), currentSetpoint);
    ffOutput = feedforward.calculate(this.getVelocity());

    // Basic Gravity Compensation:
    motorOutput = pidOutput + ffOutput;

    // Clamping:
    motorOutput = Math.min(Math.max(motorOutput, -1.0), 1.0);

    if(sharedStates.elevatorSafeToTravel){
      elevatorMotorOne.set(-pidOutput); // Negative so the elevator does not go the wrong way ¯\_(ツ)_/¯
    }
  }

  public void setElevatorPos(double height){
    this.currentSetpoint = height;
  }

  public void homeElevator(){
    currentSetpoint = 0.0;
  }
}
