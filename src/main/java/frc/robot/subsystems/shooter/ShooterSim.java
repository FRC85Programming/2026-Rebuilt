package frc.robot.subsystems.shooter;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;

public class ShooterSim extends SubsystemBase{

    private final DCMotor flywheelMotor = DCMotor.getNeoVortex(1);
    private final double gearRatioFlywheel = ShooterConstants.FLYWHEEL_GEAR_RATIO;

     private final DCMotor hoodMotor = DCMotor.getNeoVortex(1);
    private final double gearRatioHood = ShooterConstants.HOOD_GEAR_RATIO;

    private final double flywheelMOI = 0.003;
    private final double hoodMOI = 0.0025;

    private final LinearSystem<N1, N1, N1> flywheelPlant =
        LinearSystemId.createFlywheelSystem(flywheelMotor, flywheelMOI, gearRatioFlywheel);

    private final FlywheelSim flywheelSim = new FlywheelSim(flywheelPlant, flywheelMotor, gearRatioFlywheel);

    private final LinearSystem<N1, N1, N1> hoodPLant =
        LinearSystemId.createFlywheelSystem(hoodMotor, flywheelMOI, gearRatioHood);

    private final FlywheelSim hoodSim = new FlywheelSim(hoodPLant, hoodMotor, gearRatioHood);
  
    private double hoodAngleDeg = 30.0;

    private final SingleJointedArmSim hoodSim =
      new SingleJointedArmSim(
          hoodMotor,
          ShooterConstants.HOOD_GEAR_RATIO,
          hoodMOI,
          ShooterConstants.HOOD_LENGTH_METERS,
          Math.toRadians(ShooterConstants.HOOD_MIN_ANGLE),
          Math.toRadians(ShooterConstants.HOOD_MAX_ANGLE),
          true, 
          Math.toRadians(hoodAngleDeg)
      );

    private final Timer shotSpacingTimer = new Timer();

    RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
            new Translation2d(),
            new Translation2d(),
            new ChassisSpeeds(),
            new Rotation2d(),
             Units.Meters.of(0),
             Units.MetersPerSecond.of(0),
             Units.Radians.of(0)
        );


    @Override
    public void periodic() {
      hoodAngleDeg += Math.toDegrees(hoodSim.getAngularVelocityRadPerSec() * 0.02);
    }
    
    public void updateFlywheel(double flywheelVoltage, double dt) {
        flywheelSim.setInputVoltage(flywheelVoltage);
        flywheelSim.update(dt);
    }
    
    public void updateHood(double hoodVoltage, double dt) {
        hoodSim.setInputVoltage(hoodVoltage);
        flywheelSim.update(dt);
    }  
  
    public double getFlywheelRPM() {
      return flywheelSim.getAngularVelocityRPM();
    }
  
    public double getHoodAngleDeg() {
      return Math.toDegrees(hoodSim.getAngleRads());
    }

    public void generateProjectile(Pose2d pose, ChassisSpeeds velocity, Rotation2d turretAngle) {
      shotSpacingTimer.reset();
      shotSpacingTimer.start();
      // Calculate the combined shooting direction (robot + turret)
      Rotation2d shootingDirection = pose.getRotation().plus(turretAngle);
      
      Translation2d turretOffsetRotated = 
          TurretConstants.ROBOT_TO_TURRET_2D.getTranslation()
              .rotateBy(shootingDirection)
              .unaryMinus();
      
      fuelOnFly = new RebuiltFuelOnFly(
            pose.getTranslation(),
            turretOffsetRotated,
            velocity,
            shootingDirection,
            Units.Meters.of(TurretConstants.ROBOT_TO_TURRET.getZ()),
            Units.MetersPerSecond.of(getFlywheelRPM() / 6900.0 * 20.0),
            Units.Radians.of(getHoodAngleDeg()/180.0 * Math.PI)
        );

        fuelOnFly.withProjectileTrajectoryDisplayCallBack(
            (poseArray) -> Logger.recordOutput("Sim/Projectile/Trajectory", poseArray.toArray(Pose3d[]::new)),
            (poseArray) -> Logger.recordOutput("Sim/Projectile/Missed", poseArray.toArray(Pose3d[]::new))
        );

        SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);

        Logger.recordOutput("Sim/Projectile/Launched", pose);
    }

    public void cleanupPieces() {
        fuelOnFly.cleanUp();
    }

    public boolean generateProjectileIsReady() {
      return shotSpacingTimer.hasElapsed(0.25) || !shotSpacingTimer.isRunning();
    }
}
