// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.dyn4j.geometry.Rotation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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

  public static final double ROBOT_MASS = (50) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(17.6);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

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

  public static class ShooterConstants
  {
    public static final int FLYWHEEL_MOTOR_ID = 14;
    public static final int HOOD_MOTOR_ID = 15;
    public static final int FEED_MOTOR_ID = 16;
    public static final int BELT_MOTOR_ID = 24;


    public static final double FLYWHEEL_GEAR_RATIO = 1.0;
    public static final double HOOD_GEAR_RATIO = 265/25;

    // Hood limits - degrees
    public static final double HOOD_MIN_ANGLE = 40;
    public static final double HOOD_MAX_ANGLE = 80;

    public static final double SHOOTER_HEIGHT_METERS = 0.51;

    // Distance from robot center
    public static final Translation2d SHOOTER_TRANSLATION = new Translation2d(0, Units.feetToMeters(1));

    public static final double HOOD_HOME_ANGLE = 80;
    public static final double HOOD_LENGTH_METERS = 0.1;
  }

  public static class FieldConstants {
      public static final Translation3d blueHub = new Translation3d(4.620, 4.055, 1.829);
      public static final Translation3d blueFeedPosition = new Translation3d(0.7, 0.7, 0);
      public static final Translation3d redHub = new Translation3d(11.920, 4.030, 1.829);
      public static final Translation3d redFeedPosition = new Translation3d(8.0, 0.7, 0);
  }

  public static class TurretConstants {
      public static final int TURRET_MOTOR_ID = 19;

      public static final Transform3d ROBOT_TO_TURRET = new Transform3d(0, Units.feetToMeters(1), Units.inchesToMeters(1), Rotation3d.kZero);
      public static final Transform2d ROBOT_TO_TURRET_2D = new Transform2d(0, Units.feetToMeters(1), Rotation2d.kZero);

      public static final double TURRET_GEAR_RATIO = 32;

      public static final double MOUNTING_OFFSET = 90;

      public static final double TURRET_LOWER_LIMIT_DEG = -180;

      public static final double TURRET_UPPER_LIMIT_DEG = 180;
  }
  
  public static class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 18;
  }
}
