package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class OuttakeCommands extends Command {

    private final Intake intake;
    private final Shooter shooter;

    public OuttakeCommands(Intake intake, Shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;
        addRequirements(intake, shooter);
    }

    @Override
    public void execute() {
        intake.outtake();           
        shooter.intakeReverse();   
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopRoller();
        shooter.stopTrain();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}