package frc.robot.Commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeRollerCommand extends Command {
    private final Intake m_intake;
    private final boolean m_isOuttake;

    public IntakeRollerCommand(Intake intake, boolean isOuttake) {
        this.m_intake = intake;
        this.m_isOuttake = isOuttake;
        
    }

    @Override
    public void execute() {
        if (m_isOuttake) {
            m_intake.outtake();
        } else {
            m_intake.intake();
        }
    }

    @Override
    public boolean isFinished() {
      
        return true; 
    }

    
}