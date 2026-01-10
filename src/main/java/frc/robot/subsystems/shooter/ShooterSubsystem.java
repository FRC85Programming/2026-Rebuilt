package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem {
     private final SparkFlex flywheelMotor =
      new SparkFlex(ShooterConstants.FLYWHEEL_MOTOR_ID, MotorType.kBrushless);

    private final SparkFlex hoodMotor =
        new SparkFlex(ShooterConstants.HOOD_MOTOR_ID, MotorType.kBrushless);

    private final RelativeEncoder flywheelEncoder = flywheelMotor.getEncoder();
    private final RelativeEncoder hoodEncoder = hoodMotor.getEncoder();

    private final PIDController flywheelPID = new PIDController(ShooterConstants.FLYWHEEL_P, ShooterConstants.FLYWHEEL_I, ShooterConstants.FLYWHEEL_D);
    private final PIDController hoodPID = new PIDController(ShooterConstants.HOOD_P, ShooterConstants.HOOD_I, ShooterConstants.HOOD_D);

    private double convertFlywheelVelocity(double velocity) {
        return velocity / ShooterConstants.FLYWHEEL_GEAR_RATIO;
    }

    public void setFlywheelRPM(double rpm) {
        flywheelMotor.set(flywheelPID.calculate(getFlywheelRPM(), rpm));
    }

    public void stopFlywheel() {
        flywheelMotor.stopMotor();
    }

    public double getFlywheelRPM() {
        return convertFlywheelVelocity(flywheelEncoder.getVelocity());
    }

    public double getHoodAngle() {
        return hoodEncoder.getPosition();
    }

    public void setHoodAngle(double angle) {
        hoodMotor.set(hoodPID.calculate(getHoodAngle(), angle));
    }

    public boolean hoodAtAngle(double targetDeg, double tolerance) {
        return Math.abs(getHoodAngle() - targetDeg) < tolerance;
    }

    public boolean flywheelAtSpeed(double targetRPM, double tolerance) {
        return Math.abs(getFlywheelRPM() - targetRPM) < tolerance;
      }
    }

