package frc.robot.subsystems.shooter;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.ShooterConstants;

public class ShooterSim {

    DCMotor flywheelMotor = DCMotor.getNeoVortex(1);
    double gearRatio = ShooterConstants.FLYWHEEL_GEAR_RATIO;
    double flywheelMOI = 0.003; 
    double flywheelKv = 0.1;    
    double flywheelKa = 0.001;  

    double hoodAngle =  30.0; 

    LinearSystem<N1,N1,N1> flywheelPlant = LinearSystemId
    .identifyVelocitySystem(flywheelKv, flywheelKa);

    FlywheelSim flywheelSim = new FlywheelSim(
        flywheelPlant,
        flywheelMotor,
        gearRatio
    );
  
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
