package frc.robot.subsystems.shooter;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.ShooterConstants;

public class ShooterSim {

    private final DCMotor flywheelMotor = DCMotor.getNeoVortex(1);
    private final double gearRatio = ShooterConstants.FLYWHEEL_GEAR_RATIO;

    private final double flywheelMOI = 0.003;

    private final LinearSystem<N1, N1, N1> flywheelPlant =
        LinearSystemId.createFlywheelSystem(flywheelMotor, flywheelMOI, gearRatio);

    private final FlywheelSim flywheelSim = new FlywheelSim(flywheelPlant, flywheelMotor, gearRatio);
  
    private double hoodAngleDeg = 30.0;
  
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
}
