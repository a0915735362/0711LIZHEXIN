package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LL4;

public class AutoAimSpeaker extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final LL4 limelight;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

    private static final double kP = 0.02;
    private static final double kMinTurn = 0.05;
    private static final double kTolerance = 1.0; 

    public AutoAimSpeaker(CommandSwerveDrivetrain drivetrain, LL4 limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {

        double rot = 0;

        if (limelight.hasTarget()) {
            double tx = limelight.getTX();

            double rawTurn = -kP * tx;

            if (Math.abs(tx) > kTolerance) {
                rot = Math.copySign(
                    Math.max(Math.abs(rawTurn), kMinTurn),
                    rawTurn
                );
            } else {
                rot = 0;
            }
        }

        drivetrain.setControl(
            drive.withVelocityX(0)
                 .withVelocityY(0)
                 .withRotationalRate(rot * 3.0)
        );
    }

    @Override
    public boolean isFinished() {
        return limelight.hasTarget() &&
               Math.abs(limelight.getTX()) < kTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
            drive.withVelocityX(0)
                 .withVelocityY(0)
                 .withRotationalRate(0)
        );
    }
}