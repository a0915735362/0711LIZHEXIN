package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class AutoIntakeCmd extends Command {

    public enum Mode {
        EXTEND,
        BACK,
        ROLLER
    }

    private final Intake intake;

    public AutoIntakeCmd(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

     @Override
    public void execute() {

                intake.intake();   

    }


    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}