package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AlphaMechanism3d;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;

    public class TurretSim extends SubsystemBase{
        private final DCMotor turretMotor = DCMotor.getNeoVortex(1);

        SingleJointedArmSim turretSim =
                new SingleJointedArmSim(
                    turretMotor,  
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
}
