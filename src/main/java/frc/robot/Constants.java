// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
public final class Constants {

    public static final class Elevator{
        public static final int ELEVATOR_ONE_CANID = 20;
        public static final int ELEVATOR_TWO_CANID = 21;
        public static final int ELEVATOR_HOMESWITCH_PORT = 2;  //RoboRIO DIO Port for elevator's lower position

        public static final SparkMaxConfig LEFT_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig RIGHT_CONFIG = new SparkMaxConfig();
        public static final IdleMode ELEVATOR_MOTOR_IDLE_MODE = IdleMode.kBrake;
        public static final int MAX_CURRENT_LIMIT = 30;
        public static final double MAX_VELOCITY_MPS = 2;
        public static final double MAX_ELEVATOR_HEIGHT_METERS = 0;  //FIXME
        public static final double MIN_ELEVATOR_HEIGHT_METERS = 0;  //FIXME

        public static final double kP = 3.8;
        public static final double kI = 0.0;
        public static final double kD = 0.15;

        public static final double kS = 0.55;    // voltage that will slowly move the elevator
        public static final double kG = 0.3;    // voltage that will hold the elevator still
        public static final double kV = 15.0;    // known velocity / voltage used
        public static final double kA = 0.0;

        static{
            LEFT_CONFIG
                .smartCurrentLimit(MAX_CURRENT_LIMIT)
                .closedLoopRampRate(0.25)
                    .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(kP)
                .outputRange(-1, 1)

                // Slot 1 velocity control values:
                .p(0.0001, ClosedLoopSlot.kSlot1)
                .i(0.00, ClosedLoopSlot.kSlot1)
                .d(0.00, ClosedLoopSlot.kSlot1)
                .velocityFF(.5 / MAX_VELOCITY_MPS, ClosedLoopSlot.kSlot1)
                .outputRange(-.5, .5, ClosedLoopSlot.kSlot1);

            LEFT_CONFIG.
                encoder
                    .positionConversionFactor(0.01063 * 2)  // Multiply by 2 because of the second stage's mechanical advantage
                    .velocityConversionFactor(0.01063 / 60.0);

            LEFT_CONFIG.softLimit.forwardSoftLimit(MAX_CURRENT_LIMIT)
                .reverseSoftLimit(MIN_ELEVATOR_HEIGHT_METERS)
                .forwardSoftLimitEnabled(false)
                .reverseSoftLimitEnabled(false);

            RIGHT_CONFIG.
                follow(Constants.Elevator.ELEVATOR_ONE_CANID, true)
                .smartCurrentLimit(MAX_CURRENT_LIMIT)
                .closedLoopRampRate(0.25)
                    .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                
                // PID values for position control:
                .p(kP)
                .outputRange(-1, 1);

            RIGHT_CONFIG.
                encoder
                .positionConversionFactor(0.01063 * 2)  // Multiply by 2 because of the second stage's mechanical advantage
                .velocityConversionFactor(0.01063 / 60.0);
        }

        public static final double L1 = 0.0;
        public static final double L2 = 0.0;
        public static final double L3 = 0.0;
        public static final double L4 = 0.0;
    }

    public static final class Wrist{
        public static final int WRIST_CANID = 30;
        public static final int WRIST_ENCODER_PORT = 5;
        public static final double WRIST_RAW_OFFSET = 0.2410;

        public static final double CORAL_ANGLE = 15.0;
        public static final double CORAL_ANGLE_L4 = 70.0;
        public static final double WRIST_MOVEMENT_ANGLE = 16.0;

        public static final double kP = 0.01;
        public static final double kI = 0.00;
        public static final double kD = 0.0001;
        public static final double errTolerance = 0.0;

        public static final TalonFXConfiguration WRIST_CONFIG = new TalonFXConfiguration();

        static{
            WRIST_CONFIG.CurrentLimits.StatorCurrentLimit = 30;
            WRIST_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
            WRIST_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            WRIST_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        }

        public static final double L1 = 0.0;
        public static final double L2 = 0.0;
        public static final double L3 = 0.0;
        public static final double L4 = 0.0;
    }

    public static final class Manipulator{
        public static final int MAN_CANID = 31;
        public static final int CORAL_SENSOR_PORT = 0;
        public static final int ALGAE_SENSOR_PORT = 9;
        public static final double ALGAE_INTAKE_SPEED = 0.5;
        public static final double CORAL_INTAKE_SPEED = 0.2;
        public static final double OUTTAKE_SPEED = 0.10;

        public static final TalonFXConfiguration MANIPULATOR_CONFIG = new TalonFXConfiguration();

        static{
            MANIPULATOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        }
    }

    public static final class Pathplanner{
        // Load the RobotConfig from the GUI settings. You should probably
        public static RobotConfig config;
        
        static{
            try{
                config = RobotConfig.fromGUISettings();
                } catch (Exception e) {
                // Handle exception as needed
                e.printStackTrace();
                }
        }

    }
}
