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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AimAtGoal;
import frc.robot.commands.FireCommand;
import frc.robot.commands.Intake;
import frc.robot.commands.TuneShot;
import frc.robot.commands.swervedrive.auto.PathPlanToBalls;
import frc.robot.commands.swervedrive.auto.PathPlanToPath;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.BallFieldGenerator;

import java.io.File;
import java.util.Optional;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;

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

  private final TurretSubsystem turret = new TurretSubsystem();

  private final VisionSubsystem vision = new VisionSubsystem();

  BallFieldGenerator gen = new BallFieldGenerator();

  Translation2d[] balls;
  
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX() * 1)
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

    vision.setPoseSupplier(drivebase::getPose);

    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST")); 
    NamedCommands.registerCommand("Intake", new Intake(intake, () -> 0.7));
    NamedCommands.registerCommand("SmartIntake", new PathPlanToBalls(drivebase, vision, 1));
    NamedCommands.registerCommand("PathPlanToLeftRush2", new PathPlanToPath(drivebase, "LeftRush2Smart"));


    autoChooser.setDefaultOption("Left Auto", "Left");
    autoChooser.addOption("Depot+Outpost Auto", "Depot+Outpost");
    autoChooser.addOption("Left+Depot Auto", "Left+Depot");
    autoChooser.addOption("Bump Auto", "Bump");
    autoChooser.addOption("Test Auto", "Test");
    autoChooser.addOption("Double Cycle", "24BallAutoLeft");
    autoChooser.addOption("Double Cycle", "24BallAutoLeft");
    autoChooser.addOption("Left Smart Auto", "LeftSmartAuto");
    autoChooser.addOption("WORKING", "This Works Bruh");

    autoChooser.addOption("Test", "TestAuto");

    SmartDashboard.putData("Auto Selector", autoChooser);

    for (var i = 0; i < getTestBalls().length; i++) {
      //SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(getTestBalls()[i]));
    }
    
    // Passive trajectory calculation - TargetingCalculator handles turret offset internally

    // Solution involving velocity and stuff
    // turret.setAutoAngleSupplier(() -> {
    //   var solution = TargetingCalculator.calculateShot(
    //     getTarget(),
    //     drivebase.getPose(),
    //     drivebase.getFieldVelocity(),
    //     turret.getTurretAngleRads(),
    //     new TargetingCalculator.RpmConverter() {
    //       public double rpmToMps(double rpm) {
    //         return shooter.rpmToMps(rpm);
    //       }
    //       public double mpsToRpm(double mps) {
    //         return shooter.mpsToRpm(mps);
    //       }
    //     }
    //   );
    //   return solution.turretAngleDegrees;
    // });

    // Raw angle to 
  
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
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);
    Command runFlywheel = new InstantCommand(() -> shooter.setFlywheelRPM(3000));
    Command stopFlywheel = new InstantCommand(() -> shooter.setFlywheelRPM(0));
    Command angleHoodUp = new InstantCommand(() -> shooter.setHoodAngle(50.0));
    Command angleHoodDown = new InstantCommand(() -> shooter.setHoodAngle(20.0));


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
      /*Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      //driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d(0)))));
      /*driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));*/
      // driverXbox.button(1).whileTrue(new Shoot(drivebase, shooter, () -> getTarget(), false));
      driverXbox.button(1).onTrue(new PathPlanToBalls(drivebase, vision, 1, getTestBalls()));
      driverXbox.button(2).onTrue(new FireCommand(shooter, turret));
      driverXbox.button(3).whileTrue(drivebase.driveToPose(new Pose2d(14, 4, new Rotation2d())));
      driverXbox.button(4).onTrue(new InstantCommand(() -> shooter.setHoodAngle(45)));


//      driverXbox.b().whileTrue(
//          drivebase.driveToPose(
//              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
//                              );

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
      //driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      //driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(0.368, 6.000, new Rotation2d(Math.toRadians(0))))));
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightTrigger().whileTrue(new FireCommand(shooter, turret));
      driverXbox.leftTrigger()
          .onTrue(Commands.runOnce(() -> {
              shooter.startFeeding(drivebase, () -> FieldConstants.blueFeedPosition);
              turret.startFeeding(drivebase, () -> FieldConstants.blueFeedPosition);
          }))
          .onFalse(Commands.runOnce(() -> {
              shooter.startAiming(drivebase, this::getTarget);
              turret.startAiming(drivebase, this::getTarget);
          }))
          .whileTrue(new FireCommand(shooter, turret));
      driverXbox.a().onTrue(new PathPlanToBalls(drivebase, vision, -1));
      //driverXbox.pov(90).whileTrue(new InstantCommand(() -> turret.setTurretSpeed(-0.5)));
      // driverXbox.pov(90).onFalse(new InstantCommand(() -> turret.setTurretSpeed(0)));
      // driverXbox.pov(270).whileTrue(new InstantCommand(() -> turret.setTurretSpeed(0.5)));
      // driverXbox.pov(270).onFalse(new InstantCommand(() -> turret.setTurretSpeed(0)));
      driverXbox.pov(0).onTrue(new InstantCommand(() -> turret.setTurretAngle(0)));
      driverXbox.pov(180).onTrue(new InstantCommand(() -> turret.setTurretAngle(180)));
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
    shooter.startAiming(drivebase, this::getTarget);
    turret.startAiming(drivebase, this::getTarget);
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

  // I apoligize for how ugly this is :)
  public Translation2d[] getTestBalls() {
    return balls;
  }
}
