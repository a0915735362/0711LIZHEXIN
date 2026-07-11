package frc.robot.Commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;

public class IntakeExtendCommand extends IntakeCommand {
    
    private final double m_duration;
    private final Timer m_timer = new Timer();
    private final Intake m_intake;

    public IntakeExtendCommand(Intake intake, double duration) {
        super(intake, Mode.STOP); 
        this.m_intake = intake;
        this.m_duration = duration;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {

        m_intake.extend();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_duration);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stopArm();
    }
}