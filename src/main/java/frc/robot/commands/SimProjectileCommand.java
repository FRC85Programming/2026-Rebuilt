
package frc.robot.commands;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class SimProjectileCommand extends Command {
    SwerveSubsystem swerve;
    ShooterSubsystem shooter;

    public SimProjectileCommand(SwerveSubsystem swerve, ShooterSubsystem shooter) {
        this.swerve = swerve;
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        ReefscapeAlgaeOnFly algaeOnFly = new ReefscapeAlgaeOnFly(
            swerve.getPose().getTranslation(),
            new Translation2d(0.2, 0.0),
            swerve.getRobotVelocity(),
            swerve.getPose().getRotation(),
            Units.Meters.of(0.305),
            Units.MetersPerSecond.of(shooter.getSimFlywheelRPM() / 6000.0 * 20.0),
            Units.Radians.of(shooter.getSimHoodAngle()/180.0 * Math.PI)
        );

        algaeOnFly.withProjectileTrajectoryDisplayCallBack(
            (poseArray) -> Logger.recordOutput("Sim/Projectile/Trajectory", poseArray.toArray(Pose3d[]::new)),
            (poseArray) -> Logger.recordOutput("Sim/Projectile/Missed", poseArray.toArray(Pose3d[]::new))
        );

        SimulatedArena.getInstance().addGamePieceProjectile(algaeOnFly);

        Logger.recordOutput("Sim/Projectile/Launched", swerve.getPose());
    }

    @Override
    public boolean isFinished() {
        // End command immediately
        return true;
    }
}
