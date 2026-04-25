//  package frc.robot.Commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.ConsIntake;
// import frc.robot.subsystems.Intake;

// public class IntakeCommand extends Command {

//     public enum Mode {
//         EXTEND,
//         BACK,
//         ROLLER
//     }

//     private final Intake intake;
//     private final Mode mode;

//     public IntakeCommand(Intake intake, Mode mode) {
//         this.intake = intake;
//         this.mode = mode;
//         addRequirements(intake);
//     }

//      @Override
//     public void execute() {
//         switch (mode) {
//             case EXTEND:
//                 intake.extend();   
//                 break;

//             case BACK:
//                 intake.retract();  
//                 break;

//             case ROLLER:
//                 //intake.runRoller(ConsIntake.ROLLER_SPEED);
//                 break;
//         }
//     }


//     @Override
//     public void end(boolean interrupted) {
//         switch (mode) {
//             case EXTEND:
//             case BACK:
//                 intake.stopArm();
//                 break;

//             case ROLLER:
//                 intake.stopRoller();
//                 break;
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }
package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {

    public enum Mode {
        INTAKE,
        OUTTAKE,
        STOP
    }

    private final Intake intake;
    private final Mode mode;

    public IntakeCommand(Intake intake, Mode mode) {
        this.intake = intake;
        this.mode = mode;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        switch (mode) {
            case INTAKE:
                intake.intake();
                break;

            case OUTTAKE:
                intake.outtake();
                break;

            case STOP:
                intake.stopRoller();
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopRoller();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}