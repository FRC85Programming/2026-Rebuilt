package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class AimAtGoal extends Command{
    SwerveSubsystem swerve;
    TurretSubsystem turret;
    Supplier<Translation3d> goalSupplier;

    public AimAtGoal(SwerveSubsystem swerve, TurretSubsystem turret, Supplier<Translation3d> goalSupplier) {
        this.swerve = swerve;
        this.turret = turret;
        this.goalSupplier = goalSupplier;
    }

    @Override
    public void execute() {
        Pose2d robotPose = swerve.getPose();

        Translation2d turretFieldPos =
            robotPose.getTranslation()
                .plus(TurretConstants.ROBOT_TO_TURRET_2D.getTranslation().rotateBy(robotPose.getRotation()));

        Translation2d targetFieldPos =
            goalSupplier.get().toTranslation2d();

        Translation2d turretToTarget =
            targetFieldPos.minus(turretFieldPos);

        Rotation2d fieldAngle =
            new Rotation2d(turretToTarget.getX(), turretToTarget.getY());

        Rotation2d turretAngle =
            fieldAngle.minus(robotPose.getRotation());

        SmartDashboard.putNumber("Calced Turret Angle", turretAngle.getDegrees());

        turret.setTurretAngle(turretAngle.getDegrees() -90);
    }
}
