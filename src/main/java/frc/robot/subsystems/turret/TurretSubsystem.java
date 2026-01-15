package frc.robot.subsystems.turret;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase{
    private final SparkFlex turretMotor =
      new SparkFlex(TurretConstants.TURRET_MOTOR_ID, MotorType.kBrushless);

    private final AbsoluteEncoder turretEncoder = turretMotor.getAbsoluteEncoder();

    TurretSim turretSim = new TurretSim();

    PIDController angleController = new PIDController(1.0, 0, 0);

    private double targetAngle = 0.0;

    double output = 0.0;

    @Override
    public void periodic() {
        output = angleController.calculate(getTurretAngleRads(), targetAngle);

        turretMotor.set(output);

        if (RobotBase.isSimulation()) {
            turretSim.update(turretMotor.getAppliedOutput() * 12.0, 0.02);
        }

        SmartDashboard.putNumber("Target Angle", targetAngle);
        SmartDashboard.putNumber("Turret Angle", getTurretAngleRads());
        SmartDashboard.putNumber("Output Calc", output);


    }

    public void setTargetAngle(double angleDeg) {
        targetAngle = angleDeg;
    }

    public double getTurretAngleRads() {
        if (RobotBase.isSimulation()) {
            return turretSim.getTurretAngleRads();
        }
        return turretEncoder.getPosition();
    }
}
