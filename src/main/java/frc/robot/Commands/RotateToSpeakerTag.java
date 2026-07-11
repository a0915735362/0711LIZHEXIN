package frc.robot.Commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LL4;

public class RotateToSpeakerTag extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final LL4 limelight;
    private final XboxController controller;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3.0);

    private static final double kP = 0.02;
    private static final double kMinTurn = 0.05;
    private static final double kDeadband = 0.05;

    public RotateToSpeakerTag(
        CommandSwerveDrivetrain drivetrain,
        LL4 limelight,
        XboxController controller
    ) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.controller = controller;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {

        boolean hasTarget = limelight.hasTarget();
        double tx = limelight.getTX();

        double xSpeed = -controller.getLeftY();
        double ySpeed = -controller.getLeftX();

        // Deadband
        xSpeed = Math.abs(xSpeed) > kDeadband ? xSpeed : 0;
        ySpeed = Math.abs(ySpeed) > kDeadband ? ySpeed : 0;

        double rot;

        if (hasTarget && Double.isFinite(tx)) {

            double rawTurn = kP * tx;

            rawTurn = -rawTurn;

            rot = Math.copySign(
                Math.max(Math.abs(rawTurn), kMinTurn),
                rawTurn
            );

        } else {
            rot = -controller.getRightX();
        }


        double maxSpeed =
            TunerConstants.kSpeedAt12Volts.in(edu.wpi.first.units.Units.MetersPerSecond);

        double maxAngularSpeed = 3.0;

        xSpeed = xLimiter.calculate(xSpeed) * maxSpeed;
        ySpeed = yLimiter.calculate(ySpeed) * maxSpeed;
        rot = rotLimiter.calculate(rot) * maxAngularSpeed;

        drivetrain.setControl(
            drive
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(rot)
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
            drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
