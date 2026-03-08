package frc.robot.subsystems.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

    public class TurretSim extends SubsystemBase{
        SingleJointedArmSim turretSim =
                new SingleJointedArmSim(
                    DCMotor.getNeoVortex(2),  
                    TurretConstants.TURRET_GEAR_RATIO,         
                    0.02,                    
                    0.5,                
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(360),
                    false,  
                    0.0                      
                );

        public void update(double turretVoltage, double dt) {
            turretSim.setInputVoltage(turretVoltage);
            turretSim.update(dt);
        }   
        
        public double getTurretAngleRads() {
            return turretSim.getAngleRads();
        }

        public double getTurretSpeed() {
            return turretSim.getVelocityRadPerSec();
        }
}
