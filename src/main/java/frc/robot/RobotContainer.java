// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.*;
import frc.robot.Commands.IntakeCommand.Mode;
import frc.robot.subsystems.*;


import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints; 
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.ProxyCommand;

import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants;



public class RobotContainer {
    
    private final CommandXboxController operator = new CommandXboxController(3);
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final LL4 LL4 = new LL4();
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        /* Path follower */
    private final SendableChooser<Command> autoChooser;


    public RobotContainer() {

    drivetrain.setLimelight(LL4);

    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser("Tests");
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Warmup PathPlanner to avoid Java pauses
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
}

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed*0.45) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed*0.45) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate*0.6) // Drive counterclockwise with negative X (left)
            )
        );



        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );


        joystick.rightTrigger().whileTrue(drivetrain.applyRequest(() -> brake));//煞車

        joystick.x().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        NamedCommands.registerCommand("INTAKEOUT", new IntakeExtendCommand(intake, 0.5));
        NamedCommands.registerCommand("ROLLER_START_OUT", new IntakeRollerCommand(intake, true));
        NamedCommands.registerCommand("AUTOSHOOT",new ProxyCommand(() -> new AutoShoot(shooter, LL4.getDistanceMeters())).withTimeout(5.0));
        NamedCommands.registerCommand("AUTOAIM",new AutoAimSpeaker(drivetrain, LL4).withTimeout(1.2));


        // joystick.povUp().whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(0.5).withVelocityY(0))
        // );//往右移動
        // joystick.povDown().whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        // );//往左移動

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));//重置機器人面向

        drivetrain.registerTelemetry(logger::telemeterize);//送數據
        

     
      operator.a().toggleOnTrue(
    new StartEndCommand(
        intake::intake,
        intake::stopRoller,
        intake)
    );
    

        
    joystick.b().whileTrue(
        new RotateToSpeakerTag(drivetrain, LL4, operator.getHID()
        
    ));
   joystick.x().onTrue(
        new AutoAlignToTag(drivetrain, LL4)
    );

        // operator.a().toggleOnTrue(
        //     new IntakeCommand(intake, IntakeCommand.Mode.INTAKE)
        // );

        operator.b().whileTrue(
            new OuttakeCommands(intake, shooter)
        );
       
        operator.y().whileTrue(
            new Shoot(shooter, LL4, operator.getHID())
        );
        operator.x().toggleOnTrue(
            new TestAll(shooter, operator)
        );
        operator.povUp()
            .whileTrue(new InstantCommand(() -> intake.extend(), intake))
            .onFalse(new InstantCommand(() -> intake.stopArm(), intake));

        operator.povDown()
            .whileTrue(new InstantCommand(() -> intake.retract(), intake))
            .onFalse(new InstantCommand(() -> intake.stopArm(), intake));

        Pose2d targetPose = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(180));

        PathConstraints constraints = new PathConstraints(
            3.0, 
            4.0, 
            Units.degreesToRadians(540), 
            Units.degreesToRadians(720)
        );

        joystick.leftBumper().whileTrue(
            AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
        0.0
    )
);
       
    }

     
    

    public Command getAutonomousCommand() {
        
    return autoChooser.getSelected();
    
}

}