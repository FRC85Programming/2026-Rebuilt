package frc.robot.subsystems.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

    public class TurretSim extends SubsystemBase{
        private final DCMotor turretMotor = DCMotor.getNeoVortex(1);

        SingleJointedArmSim turretSim =
                new SingleJointedArmSim(
                    turretMotor,  
                    100.0,         
                    0.02,                    
                    0.5,                
                    Units.degreesToRadians(-180),
                    Units.degreesToRadians(180),
                    false,    // gravity OFF for turret
                    0.0                      
                );

        public void update(double flywheelVoltage, double dt) {
            turretSim.setInputVoltage(flywheelVoltage);
            turretSim.update(dt);
        }   
        
        public double getTurretAngleRads() {
            return turretSim.getAngleRads();
        }
}
