package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.Shooter;

public class TestAll extends Command {

    private final Shooter shooter;
    private final CommandXboxController controller;

    public TestAll(Shooter shooter, CommandXboxController controller) {
        this.shooter = shooter;
        this.controller = controller;
        addRequirements(shooter);
    }

    @Override
    public void execute() {

        double angleSpeed = controller.getLeftY();

        if (Math.abs(angleSpeed) > 0.1) {
            shooter.setAnglePosition(
                shooter.getAngle() + angleSpeed * 0.2
            );
        }

        shooter.setShooterRPM(5000);

        if (controller.getRightTriggerAxis() > 0.5) {
            shooter.setIntakeTrain(-0.7);
            shooter.setShooterTrain(0.8);
        } else {
            shooter.setIntakeTrain(0.0);
            shooter.setShooterTrain(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
        shooter.stopTrain();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}