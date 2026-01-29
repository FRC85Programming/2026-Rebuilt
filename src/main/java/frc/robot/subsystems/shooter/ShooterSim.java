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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSim extends SubsystemBase{

    private final DCMotor flywheelMotor = DCMotor.getNeoVortex(1);
    private final double gearRatio = ShooterConstants.FLYWHEEL_GEAR_RATIO;

    private final double flywheelMOI = 0.003;

    private final LinearSystem<N1, N1, N1> flywheelPlant =
        LinearSystemId.createFlywheelSystem(flywheelMotor, flywheelMOI, gearRatio);

    // THIS NAME NEEDS TO BE CHANGED
    private final FlywheelSim flywheelSim = new FlywheelSim(flywheelPlant, flywheelMotor, gearRatio);
  
    private double hoodAngleDeg = 30.0;

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


    public void update(double flywheelVoltage, double dt) {
        flywheelSim.setInputVoltage(flywheelVoltage);
        flywheelSim.update(dt);
    }  
  
    public double getFlywheelRPM() {
      return flywheelSim.getAngularVelocityRPM();
    }
  
    public double getHoodAngleDeg() {
      return hoodAngleDeg;
    }

    public void setHoodAngle(double angle) {
      hoodAngleDeg = angle;
    }

    public void generateProjectile(Pose2d pose, ChassisSpeeds velocity) {
      shotSpacingTimer.reset();
      shotSpacingTimer.start();
      fuelOnFly = new RebuiltFuelOnFly(
            pose.getTranslation(),
            new Translation2d(0.0, 0.0),
            velocity,
            pose.getRotation(),
            Units.Meters.of(Constants.ShooterConstants.SHOOTER_HEIGHT_METERS),
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
