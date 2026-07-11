package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.ShooterLookup;
import frc.robot.ShooterLookup.Point;

public class AutoShoot extends Command {

    private final Shooter shooter;
    private final double distanceM;

    private double targetRPM;
    private double targetAngle;

    public AutoShoot(Shooter shooter, double distanceM) {
        this.shooter = shooter;
        this.distanceM = distanceM;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        Point p = ShooterLookup.sample(distanceM);

        targetRPM = p.rpm;
        targetAngle = p.pitchRot;
    }

    @Override
    public void execute() {

        shooter.setShooterRPM(targetRPM);
        shooter.setAnglePosition(targetAngle);

      
        shooter.feedForward();
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