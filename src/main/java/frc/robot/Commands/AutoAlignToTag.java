package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LL4;

/**
 * Auto-align the robot to any visible fiducial (AprilTag) using the limelight's TX value.
 * Pressing the bound button will start this command; it finishes when the PID is at setpoint
 * or when the limelight no longer sees a target.
 */
public class AutoAlignToTag extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final LL4 limelight;

    private final PIDController rotPID = new PIDController(0.1, 0.0, 0.002);
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

    public AutoAlignToTag(CommandSwerveDrivetrain drivetrain, LL4 limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;

        rotPID.setTolerance(0.2); // degrees of horizontal error

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        if (!limelight.hasTarget()) {
            // no target: ensure we stop rotating
            drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            return;
        }

        double tx = limelight.getTX(); // horizontal offset in degrees (Limelight convention)

        double rot = rotPID.calculate(tx, 0.0);

        // limit rotational command to a safe range (radians/sec used by drivetrain API in this project)
        rot = clamp(rot, -2.5, 2.5);

        // avoid too-small commands that never overcome static friction; ensure a small minimum
        if (Math.abs(rot) < 0.05) {
            rot = Math.copySign(0.05, rot);
        }

        drivetrain.setControl(
            drive.withVelocityX(0).withVelocityY(0).withRotationalRate(rot+0.1)
        );
    }

    @Override
    public boolean isFinished() {
        // finish when on target or no target anymore
        return rotPID.atSetpoint() || !limelight.hasTarget();
    }

    @Override
    public void end(boolean interrupted) {
        // stop the drivetrain rotation when command ends
        drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}