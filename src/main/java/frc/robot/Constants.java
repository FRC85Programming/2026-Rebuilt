// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
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

  public static final double ROBOT_MASS = (115) * 0.453592; // lots of lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
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
    public static final int HOOD_MOTOR_ID = 15;


    public static final double FLYWHEEL_GEAR_RATIO = 1;
    public static final double HOOD_GEAR_RATIO = 265.0/25.0;

    // Hood limits - degrees
    public static final double HOOD_MIN_ANGLE = 40;
    public static final double HOOD_MAX_ANGLE = 75;

    public static final double SHOOTER_HEIGHT_METERS = 0.51;

    // Distance from robot center
    public static final Translation2d SHOOTER_TRANSLATION = new Translation2d(Units.inchesToMeters(-5.5), 0);

    public static final double HOOD_HOME_ANGLE = 75;
    public static final double HOOD_LENGTH_METERS = 0.1;
  }

  public static class FieldConstants {
      public static final Translation3d blueHub = new Translation3d(4.620, 4.035, 1.829);
      public static final Translation3d blueFeedPosition = new Translation3d(2.6, 0.0, 1.829);
      public static final Translation3d redHub = new Translation3d(11.915, 4.035, 1.829);
      public static final Translation3d redFeedPosition = new Translation3d(13.930, 0.0, 1.829);

      // Y range of the obstacle that blocks direct feed shots
      public static final double FEED_BLOCK_Y_MIN = 3.25;
      public static final double FEED_BLOCK_Y_MAX = 4.65;
      // Tangent of the deflection angle used to offset the feed target around the obstacle.
      // Positive value deflects above the midpoint upward and below the midpoint downward.
      // Negate FEED_ANGLE_SCALE_RED if red requires the opposite deflection direction.
      public static final double FEED_ANGLE_SCALE_BLUE = 0.5;
      public static final double FEED_ANGLE_SCALE_RED  = 0.5;

      // Saftey poses
      public static final Translation2d blueLeftEnterAllianceZonePose = new Translation2d(5.825, 7.500);
      public static final Translation2d blueRightEnterAllianceZonePose = new Translation2d(5.825, 0.600);

      public static final Pose2d BLUE_LEFT_START_POSE = new Pose2d(4.324, 7.465, new Rotation2d(Math.toRadians(-90)));
      public static final Pose2d BLUE_RIGHT_START_POSE = new Pose2d(4.324, 0.654, new Rotation2d(Math.toRadians(90)));
      public static final Pose2d RED_RIGHT_START_POSE = new Pose2d(12.217, 7.465, new Rotation2d(Math.toRadians(-90)));
      public static final Pose2d RED_LEFT_START_POSE = new Pose2d(12.217, 0.624, new Rotation2d(Math.toRadians(90)));
      public static final Pose2d BLUE_CENTER_STARTING_POSE = new Pose2d(3.622, 4.970, new Rotation2d(Math.toRadians(180)));
      public static final Pose2d RED_CENTER_STARTING_POSE = new Pose2d(12.889, 3.080, new Rotation2d(Math.toRadians(0)));
      public static final Pose2d BLUE_HUB_STARTING_POSE = new Pose2d(3.583, 3.996, new Rotation2d(Math.toRadians(180)));
      public static final Pose2d RED_HUB_STARTING_POSE = new Pose2d(12.967, 4.016, new Rotation2d(Math.toRadians(0)));

      /**
       * X boundary near the blue alliance structure. When the robot is below this
       * value and moving in the positive-X direction it is entering the protected zone
       * and the shooter/turret should be idled automatically.
       */
      public static final double SHOOTER_IDLE_ZONE_BLUE_BOTTOM = 4.0;

      public static final double SHOOTER_IDLE_ZONE_BLUE_TOP = 5.3;

      /**
       * X boundary near the red alliance structure. When the robot is above this
       * value and moving in the negative-X direction it is entering the protected zone
       * and the shooter/turret should be idled automatically.
       */
      public static final double SHOOTER_IDLE_ZONE_RED_BOTTOM = 11.3;

      public static final double SHOOTER_IDLE_ZONE_RED_TOP = 12.55;



      /**
       * Minimum field-relative X speed (m/s) required to consider the robot
       * "approaching" a boundary.  Prevents false triggers from slow drift.
       */
      public static final double SHOOTER_IDLE_APPROACH_VEL = 0.3;
  }

  public static class TurretConstants {
      public static final int TURRET_MOTOR_ID = 19;

      public static final Transform3d ROBOT_TO_TURRET = new Transform3d(Units.inchesToMeters(-5.5), 0, Units.inchesToMeters(1), Rotation3d.kZero);
      public static final Transform2d ROBOT_TO_TURRET_2D = new Transform2d(Units.inchesToMeters(-5.5),0, Rotation2d.kZero);

      public static final double TURRET_GEAR_RATIO = 32;

      public static final double MOUNTING_OFFSET = -180;

      public static final double TURRET_LOWER_LIMIT_DEG = -190;


      public static final double TURRET_UPPER_LIMIT_DEG = 168;

      // MEASURED IN RPM
      public static final double TURRET_SPEED_SAFEZONE = 1000;
  }
  
  public static class IntakeConstants {
    public static final int ROLLER_MOTOR_LEFT_ID = 18;

    public static final int ROLLER_MOTOR_RIGHT_ID = 50;

    public static final int PIVOT_MOTOR_ID = 25;

    // The position (in motor rotations) in which the intake is in the down position
    public static final double INTAKE_DOWN_POSITION = -9.4;

    public static final double INTAKE_STOW_POSITION = -3;

    public static final double INTAKE_DEPOT_POSITION = -8.7;

    // The position (in motor rotations) in which the intake is fully up/retracted
    public static final double INTAKE_UP_POSITION = 0;
  }

  public static class ObstacleAlignmentConstants {

    /** How close (meters, measured in X) the robot must be to an obstacle before corrections begin. */
    public static final double DETECTION_RANGE = 1.5;

    // --- Trench boxes ---
    public static final double TRENCH1_X_MIN  = 4.0;
    public static final double TRENCH1_X_MAX  = 5.2;
    public static final double TRENCH1_Y_MIN  = 6.5;
    public static final double TRENCH1_Y_MAX  = 8.1;
    /** Configurable center Y the robot tries to hold while traversing Trench 1. */
    public static final double TRENCH1_TARGET_Y = 7.4;

    public static final double TRENCH2_X_MIN  = 4.0;
    public static final double TRENCH2_X_MAX  = 5.2;
    public static final double TRENCH2_Y_MIN  = 0.0;
    public static final double TRENCH2_Y_MAX  = 1.5;
    /** Configurable center Y the robot tries to hold while traversing Trench 2. */
    public static final double TRENCH2_TARGET_Y = 0.65;

    // --- Y-centering PID ---
    public static final double Y_KP = 3.0;
    public static final double Y_KI = 0.0;
    public static final double Y_KD = 0.05;
  }

  public static class IndexerConstants {
    public static final int INDEXER_MOTOR_ID = 16;
    public static final int BELT_MOTOR_ID = 24;
  }

  public static class ClimberConstants {
    public static final int CLIMB_MOTOR_ID = 50;

    public static final double CLIMBER_UP_POSITION = 83.5;

    public static final double CLIMBER_CLIMB_POSITION = 50;

  }
}
