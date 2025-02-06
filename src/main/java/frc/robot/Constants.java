// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.atomic.AtomicBoolean;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.drivers.CANDeviceId;
import frc.lib.subsystems.ServoMotorSubsystemConfig;
import frc.robot.dtos.EncoderConfig;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  public static final String kCanBusCanivore = "Canivore";

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.
  public static final double DISTANCE_PER_PULSE = 360.0/2048.0;

  // Swerve auto constants
  public static final double ALIGN_TOLERANCE = 1.0;
  public static final double ROTATION_SPEED = 0.5;

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class GripperConstants {
    public static final int GRIPPER_PORT1 = 1;
    public static final int GRIPPER_PORT2 = 2;
  }

  public static class ClimbConstants {
    public static final CANDeviceId kClimbTalonCanId = new CANDeviceId(0,kCanBusCanivore);
    public static final int CLIMB_PORT1 = 4;
    public static final EncoderConfig CLIMB_ENCODER_CONFIG = new EncoderConfig(1,1,1 );
    public static final double kForwardMaxPositionRotations = 132;
    private static class ClimbPoses {
      private static final double CLIMB1_START_POSE = 0;
      private static final double CLIMB1_TARGET_POSE = 360;
      private static final double CLIMB2_START_POSE = 0;
      private static final double CLIMB2_TARGET_POSE = 1080;
      private static final double CLIMB3_START_POSE = 0;
      private static final double CLIMB3_TARGET_POSE = 180;
    }
  
    public static final double CLIMB_START_POSE = ClimbPoses.CLIMB1_START_POSE;
    public static final double CLIMB_TARGET_POSE = ClimbPoses.CLIMB1_TARGET_POSE;
    
  }
  public static final ServoMotorSubsystemConfig kClimberConfig = new ServoMotorSubsystemConfig();
    static {
        kClimberConfig.name = "Climber";
        kClimberConfig.talonCANID = new CANDeviceId(24, kCanBusCanivore);
        kClimberConfig.kMaxPositionUnits = ClimbConstants.kForwardMaxPositionRotations;
        kClimberConfig.kMinPositionUnits = 0.0;
        kClimberConfig.fxConfig.Slot0.kP = 1.0 * 12.0;
        kClimberConfig.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        kClimberConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kClimberConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        kClimberConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        // kClimberConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ClimbConstants.kForwardMaxPositionRotations
        //         - Constants.ClimbConstants.kPositionToleranceRotations;
        kClimberConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

        kClimberConfig.fxConfig.Audio.BeepOnBoot = false;
        kClimberConfig.fxConfig.Audio.BeepOnConfig = false;
        kClimberConfig.unitToRotorRatio = 1.0;

        kClimberConfig.fxConfig.CurrentLimits.StatorCurrentLimit = 150.0;
        kClimberConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        // kClimberConfig.fxConfig.ClosedLoopRamps = makeDefaultClosedLoopRampConfig();
        // kClimberConfig.fxConfig.OpenLoopRamps = makeDefaultOpenLoopRampConfig();
        kClimberConfig.momentOfInertia = 0.05;
    }



  public static class ElevatorConstants{
    public static final double kElevatorPositioningToleranceInches = 0.03;
  }
  public static final AprilTagFieldLayout kAprilTagLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

  public static final double kFieldWidthMeters = kAprilTagLayout.getFieldWidth(); // distance between field walls,
                                                                                    // 8.211m
  public static final double kFieldLengthMeters = kAprilTagLayout.getFieldLength(); // distance between driver station
                                                                                      // walls, 16.541m
}
