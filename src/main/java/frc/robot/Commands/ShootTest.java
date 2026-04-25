package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootTest extends Command {

  private final Shooter shooter;
  private final XboxController controller;

  public ShootTest(Shooter shooter, XboxController controller) {
    this.shooter = shooter;
    this.controller = controller;
    addRequirements(shooter);
  }

  @Override
  public void execute() {

    shooter.setShooterDuty(0.05);

    if (controller.getRightTriggerAxis() > 0.5) {
      shooter.setIntakeTrain(-0.1);
      shooter.setShooterTrain(0.1);
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