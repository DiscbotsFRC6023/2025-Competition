// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotStates;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private final TalonFX wristMotor;
  private RobotStates sharedStates;
  private DutyCycleEncoder wristEncoder;
  private PIDController wristController;
  private double pidOutput = 0.0;
  private double commandedAngle = 0.0;
  private boolean eStop = false;

  public Wrist(RobotStates sharedStates) {
    wristMotor = new TalonFX(Constants.Wrist.WRIST_CANID);
    wristController = new PIDController(Constants.Wrist.kP, Constants.Wrist.kI, Constants.Wrist.kD);
    wristController.setTolerance(Constants.Wrist.errTolerance);
    wristEncoder = new DutyCycleEncoder(Constants.Wrist.WRIST_ENCODER_PORT);
    wristMotor.getConfigurator().apply(Constants.Wrist.WRIST_CONFIG);
    this.sharedStates = sharedStates;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist/DEG Position:", getAbsWristPosInDegrees());
    SmartDashboard.putBoolean("Wrist/ENC Connected:", wristEncoder.isConnected());
    SmartDashboard.putNumber("Wrist/RAW Position:", wristEncoder.get());
    SmartDashboard.putNumber("Wrist/Commanded Degrees:", commandedAngle);
    SmartDashboard.putNumber("Wrist/PID Output:", pidOutput);
    SmartDashboard.putBoolean("Wrist/SafeForTravel", this.safeForTravel());
    SmartDashboard.putBoolean("Wrist/ESTOP:", eStop);
    
    sharedStates.elevatorSafeToTravel = this.safeForTravel();

    // Makes sure that reenabling the robot will not spring wrist back to last position
    if(DriverStation.isDisabled()){
      commandedAngle = getAbsWristPosInDegrees();
    } else {
      this.driveWrist();
    }
  }

  public double getAbsWristPos(){
    return wristEncoder.get() - Constants.Wrist.WRIST_RAW_OFFSET;
  }

  public double getRawWristPos(){
    return wristEncoder.get();
  }

  public double getAbsWristPosInDegrees(){
    return getAbsWristPos() * 360;
  }

  public void manualWrist(double speed){
    wristMotor.set(speed);
  }

  // This takes a parameter to move the wrist forward from zero by "angle" degrees
  private void driveWrist(){
    if(wristEncoder.isConnected() && !eStop){
      pidOutput = wristController.calculate(getAbsWristPosInDegrees(), commandedAngle);
    } else {
      pidOutput = 0.0;
    }
    wristMotor.set(pidOutput);
  }

  public void setWristPos(double angle){
    commandedAngle = angle;
  }

  public void stopAll(){
    wristMotor.stopMotor();
    eStop = true;
  }

  public boolean safeForTravel(){
    if((getAbsWristPosInDegrees() >= 15.5 && getAbsWristPosInDegrees() <= 21.0) || getAbsWristPosInDegrees() >= 90.0){ //FIXME: match these values to actual robot
      return true;
    }
    return false;
  }

  public void homeWrist(){
    this.setWristPos(Constants.Wrist.homeDegrees);
  }
}
