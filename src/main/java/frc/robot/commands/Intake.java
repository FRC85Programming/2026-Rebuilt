package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class Intake extends Command{
    private IntakeSubsystem intake;

    public Intake(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.deployIntake();
        intake.runRollers();
    }


    @Override
    public void end(boolean interrupted) {
    }
}
