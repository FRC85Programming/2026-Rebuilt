package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class Intake extends Command{
    private IntakeSubsystem intake;
    private DoubleSupplier speed;

    public Intake(IntakeSubsystem intake, DoubleSupplier speed) {
        this.intake = intake;
        this.speed = speed;
        
    }

    @Override
    public void initialize() {
        intake.runIntake(speed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        intake.runIntake(0);
    }
}
