
// package frc.robot.commands;

// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.ShooterLookup;
// import frc.robot.subsystems.LL4;
// import frc.robot.subsystems.Shooter;

// public class Shoot extends Command {

//   private final Shooter shooter;
//   private final LL4 ll4;
//   private final XboxController drive;

//   private static final double kFeedTrigger = 0.3;
//   private static final double kReverseTrigger = 0.4;

//   private static final double kShooterFallbackSpeed = 0.5;
//   private static final double kTrainFeedSpeed = -0.7;
//   private static final double kIntakeFeedSpeed = 0.55;

//   private static final double kTrainReverseSpeed = 0.7;
//   private static final double kIntakeReverseSpeed = 0.7;

//   private static final double kMinValidDist = 0.05;

//   private double lastTargetRpm = 2200.0;
//   private double lastTargetPitchRot = 0.0;

//   public Shoot(Shooter shooter, LL4 ll4, XboxController drive) {
//     this.shooter = shooter;
//     this.ll4 = ll4;
//     this.drive = drive;

//     addRequirements(shooter);
//   }

//   @Override
//   public void execute() {
//     double dist = ll4.getDistanceMeters();

//     if (Double.isFinite(dist) && dist > kMinValidDist) {

//       ShooterLookup.Point sp = ShooterLookup.sample(dist);

//       lastTargetRpm = sp.rpm;
//       lastTargetPitchRot = sp.pitchRot;

//       shooter.setShooterRPM(lastTargetRpm);
//       shooter.setAnglePosition(lastTargetPitchRot);

//       System.out.printf("DIST=%.2f RPM=%.1f PITCH=%.3f%n",
//           dist, lastTargetRpm, lastTargetPitchRot);

//     } else {
//       shooter.setShooterDuty(kShooterFallbackSpeed);
//     }

//     if (drive.getRightTriggerAxis() > kFeedTrigger) {
//       shooter.setShooterTrain(kTrainFeedSpeed);
//       shooter.setIntakeTrain(kIntakeFeedSpeed);

//     } else if (drive.getLeftTriggerAxis() > kReverseTrigger) {
//       shooter.setShooterTrain(kTrainReverseSpeed);
//       shooter.setIntakeTrain(kIntakeReverseSpeed);

//     } else {
//       shooter.setShooterTrain(0.0);
//       shooter.setIntakeTrain(0.0);
//     }
//   }

//   @Override
//   public void end(boolean interrupted) {
//     shooter.stopShooter();
//     shooter.stopTrain();
//   }

//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShooterLookup;
import frc.robot.subsystems.LL4;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {

  private final Shooter shooter;
  private final LL4 ll4;
  private final XboxController drive;

  private static final double kFeedTrigger = 0.3;
  private static final double kReverseTrigger = 0.4;

  private static final double kMinValidDist = 0.05;


  private static final double kFallbackRPM = 3800.0;

  
  private static final double kRPMTolerance = 150.0;

  private static final double kTrainFeedSpeed = 0.6;
  private static final double kIntakeFeedSpeed = 0.5;

  private static final double kTrainReverseSpeed = -0.6;
  private static final double kIntakeReverseSpeed = -0.6;

  private double targetRPM = kFallbackRPM;
  private double targetAngle = -0.6;

  public Shoot(Shooter shooter, LL4 ll4, XboxController drive) {
    this.shooter = shooter;
    this.ll4 = ll4;
    this.drive = drive;

    addRequirements(shooter);
  }

  @Override
  public void execute() {

   
    double dist = ll4.getDistanceMeters();

    if (Double.isFinite(dist) && dist > kMinValidDist) {
      ShooterLookup.Point sp = ShooterLookup.sample(dist);

      targetRPM = sp.rpm;
      targetAngle = sp.pitchRot;

      System.out.printf("DIST=%.2f RPM=%.1f ANG=%.3f%n",
          dist, targetRPM, targetAngle);
    }

    
    shooter.setShooterRPM(targetRPM);
    shooter.setAnglePosition(targetAngle);

  
    if (drive.getRightTriggerAxis() > kFeedTrigger) {

        shooter.setShooterTrain(kTrainFeedSpeed);
        shooter.setIntakeTrain(-kIntakeFeedSpeed);
      
    } else if (drive.getLeftTriggerAxis() > kReverseTrigger) {


    } else {
      shooter.stopTrain();
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