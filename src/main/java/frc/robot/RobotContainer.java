// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.FireCommand;
import frc.robot.commands.Intake;
import frc.robot.commands.TuneShot;
import frc.robot.commands.swervedrive.auto.PathPlanToBalls;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.BallFieldGenerator;

import java.io.File;
import java.util.Optional;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));
  private final ShooterSubsystem shooter = new ShooterSubsystem();

  private final IntakeSubsystem intake = new IntakeSubsystem();

  private final IndexerSubsystem indexer = new IndexerSubsystem();

  private final TurretSubsystem turret = new TurretSubsystem();

  //private final VisionSubsystem vision = new VisionSubsystem();

  private final ClimberSubsystem climber = new ClimberSubsystem();


  BallFieldGenerator gen = new BallFieldGenerator();

  Translation2d[] balls;
  
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * 1,
                                                                () -> driverXbox.getLeftX() * 1)
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));
  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    balls = gen.getBalls();

    //vision.setPoseSupplier(drivebase::getPose);

    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    // Basic intaking command (same as the one bound to a button)
    Command intakeCommand = new Intake(intake);
    NamedCommands.registerCommand("Start Intake", intakeCommand);
    //NamedCommands.registerCommand("Stop Intake", Commands.runOnce(() -> intakeCommand.cancel()));

    // Change shooter states and start shooting
    Command shootCommand = new SequentialCommandGroup(new InstantCommand(() -> shooter.startAiming(drivebase, () -> getTarget())), 
                                                      new InstantCommand(() -> turret.startAiming(drivebase, () -> getTarget())),
                                                      new FireCommand(shooter, indexer, turret, intake));
    NamedCommands.registerCommand("Start Shooting", shootCommand);


    // Change shooter states and stop shooting — requiring shooter+turret interrupts the running FireCommand,
    // which then calls FireCommand.end() to stop the flywheel and indexer automatically.
    Command stopShootingCommand = Commands.runOnce(() -> {
        shooter.stopAiming();
        turret.stopAiming();
    }, shooter, turret);

    NamedCommands.registerCommand("Aim Turret", new InstantCommand(() -> turret.startAiming(drivebase, () -> getTarget())));

    NamedCommands.registerCommand("Stop Shooting", stopShootingCommand);

    NamedCommands.registerCommand("Intake Up", new InstantCommand(() -> intake.toggleStowedUp()));

    NamedCommands.registerCommand("Intake Depot", new SequentialCommandGroup(new InstantCommand(() -> intake.deployIntakeDepot()), new InstantCommand(() -> intake.runRollers())));

    //NamedCommands.registerCommand("Climb", new Climb(ClimberSubsystem));

    // Movement commands
    //NamedCommands.registerCommand("SmartIntakeBlueLeft", new PathPlanToBalls(drivebase, vision, 5.64, 8.43, 4, 7.5));
    NamedCommands.registerCommand("DriveToBlueLeftShoot", drivebase.driveToPose(new Pose2d(3.625, 7.406, new Rotation2d(Math.toRadians(180)))));


    autoChooser.addOption("LeftDoubleRush", "LeftDoubleRush");
    autoChooser.addOption("LeftBumpRush", "LeftBumpRush");
    autoChooser.addOption("Left+Depot", "Left+Depot");
    autoChooser.addOption("RightDoubleRush", "RightDoubleRush");
    autoChooser.addOption("RightBumpRush", "RightBumpRush");
    autoChooser.addOption("Right+Outpost", "Right+Outpost");
    autoChooser.addOption("Center", "Center");

    SmartDashboard.putData("Auto Selector", autoChooser);

    for (var i = 0; i < getTestBalls().length; i++) {
      //SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(getTestBalls()[i]));
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Maybe try this at some point?
    // Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);

    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocityKeyboard);
      drivebase.setVelocitySupplier(driveAngularVelocityKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
      drivebase.setVelocitySupplier(driveAngularVelocity);
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }

    if (Robot.isSimulation())
    {
      //driverXbox.button(1).whileTrue(new PathPlanToBalls(drivebase, vision, getTestBalls(), 5.2, 8.43, 3.9, 7.5));
      driverXbox.button(2).onTrue(new FireCommand(shooter, indexer, turret, intake));
      driverXbox.button(3).whileTrue(drivebase.driveToPose(new Pose2d(14, 4, new Rotation2d())));
      driverXbox.button(4).onTrue(new InstantCommand(() -> shooter.setHoodAngle(45)));
    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      // TODO: Configure this pose to a better position/use apriltags
      //driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(0.368, 6.000, new Rotation2d(0)))));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3.690, 7.377, new Rotation2d(0)))));
      //driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

      // Right Trigger - Shoot based on current mode
      driverXbox.rightTrigger().whileTrue(new FireCommand(shooter, indexer, turret, intake));

      // Left Trigger - Intake
      driverXbox.leftTrigger().whileTrue(new Intake(intake));
      driverXbox.rightBumper().onTrue(new InstantCommand(() -> intake.toggleStowedUp()));

      driverXbox.pov(0).onTrue(new InstantCommand(() -> climber.climberUp()));
      driverXbox.pov(180).onTrue(new InstantCommand(() -> climber.climberDown()));

      // X - Switch shooter to idle mode
      driverXbox.x().onTrue(Commands.runOnce(() -> {
              shooter.stopAiming();
              turret.stopAiming();
          }));

      // B - Start aiming in case of failure to auto init
      driverXbox.b().onTrue(Commands.runOnce(() -> {
              shooter.startAiming(drivebase, () -> getTarget());
              turret.startAiming(drivebase, () -> getTarget());
          }));

      // A - Start feeding mode
      driverXbox.a().onTrue(Commands.runOnce(() -> {
              shooter.startFeeding(drivebase, () -> getFeedTarget());
              turret.startFeeding(drivebase, () -> getFeedTarget());
          }));

      // Quick inputs for spinning turret - TEST ONLY
      //driverXbox.pov(0).onTrue(new InstantCommand(() -> turret.setTurretAngle(0)));
      //driverXbox.pov(180).onTrue(new InstantCommand(() -> turret.setTurretAngle(180)));

      // Idle shooter/turret when inside either trench idle box.
      Trigger inIdleZone = new Trigger(() -> {
        double x = drivebase.getPose().getX();
        double y = drivebase.getPose().getY();
        boolean inTrench = (y >= Constants.ObstacleAlignmentConstants.TRENCH1_Y_MIN
                         && y <= Constants.ObstacleAlignmentConstants.TRENCH1_Y_MAX)
                        || (y >= Constants.ObstacleAlignmentConstants.TRENCH2_Y_MIN
                         && y <= Constants.ObstacleAlignmentConstants.TRENCH2_Y_MAX);
        boolean inBlueBox = x > Constants.FieldConstants.SHOOTER_IDLE_ZONE_BLUE_BOTTOM
                         && x < Constants.FieldConstants.SHOOTER_IDLE_ZONE_BLUE_TOP;
        boolean inRedBox  = x > Constants.FieldConstants.SHOOTER_IDLE_ZONE_RED_BOTTOM
                         && x < Constants.FieldConstants.SHOOTER_IDLE_ZONE_RED_TOP;
        return inTrench && (inBlueBox || inRedBox);
      });
      inIdleZone.onTrue(Commands.runOnce(() -> {
        shooter.stopAiming();
        turret.stopAiming();
      }));

      // Re-activate aiming when the robot crosses into the alliance from the idle box.
      Trigger inAllianceZone = new Trigger(() -> {
        double x = drivebase.getPose().getX();
        double y = drivebase.getPose().getY();
        boolean inTrench = (y >= Constants.ObstacleAlignmentConstants.TRENCH1_Y_MIN
                         && y <= Constants.ObstacleAlignmentConstants.TRENCH1_Y_MAX)
                        || (y >= Constants.ObstacleAlignmentConstants.TRENCH2_Y_MIN
                         && y <= Constants.ObstacleAlignmentConstants.TRENCH2_Y_MAX);
        boolean inBlueAlliance = x < Constants.FieldConstants.SHOOTER_IDLE_ZONE_BLUE_BOTTOM;
        boolean inRedAlliance  = x > Constants.FieldConstants.SHOOTER_IDLE_ZONE_RED_TOP;
        return inTrench && (inBlueAlliance || inRedAlliance);
      });
      inAllianceZone.onTrue(Commands.runOnce(() -> {
        shooter.startAiming(drivebase, () -> getTarget());
        turret.startAiming(drivebase, () -> getTarget());
      }));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand(autoChooser.getSelected());
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public void teleopInit()
  {
    /*shooter.startAiming (drivebase, this::getTarget);
    turret.startAiming(drivebase, this::getTarget);*/
  }

  public Translation3d getTarget() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
        if (alliance.get() == Alliance.Red) {
            return Constants.FieldConstants.redHub;
        }
        else {
            return Constants.FieldConstants.blueHub;
        }
    }
    else {
        return new Translation3d();
    }
  }

  public Translation3d getFeedTarget() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    double robotX = drivebase.getPose().getX();
    double robotY = drivebase.getPose().getY();

    Translation3d base = (alliance.isPresent() && alliance.get() == Alliance.Red)
        ? Constants.FieldConstants.redFeedPosition
        : Constants.FieldConstants.blueFeedPosition;

    double targetY = robotY;
    double yMin = Constants.FieldConstants.FEED_BLOCK_Y_MIN;
    double yMax = Constants.FieldConstants.FEED_BLOCK_Y_MAX;

    if (robotY >= yMin && robotY <= yMax) {
      double midpoint = (yMin + yMax) / 2.0;
      double sign     = (robotY >= midpoint) ? 1.0 : -1.0;
      double xDist    = Math.abs(base.getX() - robotX);
      double scale    = (alliance.isPresent() && alliance.get() == Alliance.Red)
                        ? Constants.FieldConstants.FEED_ANGLE_SCALE_RED
                        : Constants.FieldConstants.FEED_ANGLE_SCALE_BLUE;
      targetY = robotY + sign * xDist * scale;
    }

    return new Translation3d(base.getX(), targetY, base.getZ());
  }

  // I apoligize for how ugly this is :)
  public Translation2d[] getTestBalls() {
    return balls;
  }
}
