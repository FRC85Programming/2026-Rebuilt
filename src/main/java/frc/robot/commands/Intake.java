package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem.Animation;

public class Intake extends Command{
    private IntakeSubsystem intake;
    private LEDSubsystem leds;

    public Intake(IntakeSubsystem intake, LEDSubsystem leds) {
        this.intake = intake;
        this.leds = leds;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.deployIntake();
        intake.runRollers();
        leds.setAnimation(Animation.ALLIANCE_SPECIFIC);
    }


    @Override
    public void end(boolean interrupted) {
    }
}
