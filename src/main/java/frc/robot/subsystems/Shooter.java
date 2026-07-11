
package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConsIntake;
import frc.robot.Constants.ConsShooter;

public class Shooter extends SubsystemBase {

    //TalonFX 
    private final TalonFX flywheelMain =
        new TalonFX(ConsShooter.FLYWHEEL_MAIN_ID, ConsIntake.KcanBusName);

    private final TalonFX flywheelFollower =
        new TalonFX(ConsShooter.FLYWHEEL_FOLLOWER_ID, ConsIntake.KcanBusName);

    private final TalonFX shooterTrain =
        new TalonFX(ConsShooter.SHOOTER_TRAIN_ID);

    //SparkMAX
    private final SparkMax intakeTrain =
        new SparkMax(ConsShooter.INTAKE_TRAIN_ID, MotorType.kBrushless);

    private final SparkMax angle =
        new SparkMax(ConsShooter.ANGLE_ID, MotorType.kBrushless);

    private final RelativeEncoder angleEncoder = angle.getEncoder();
    private final SparkClosedLoopController anglePID = angle.getClosedLoopController();

    //Control Requests
    private final VoltageOut flywheelVoltageReq = new VoltageOut(0);
    private final VelocityVoltage flywheelVelocityReq =
        new VelocityVoltage(0).withSlot(0);

    private final VoltageOut shooterTrainVoltageReq = new VoltageOut(0);

    public Shooter() {

        flywheelMain.getConfigurator().apply(ConsShooter.FLYWHEEL_MAIN_CONFIG);
        flywheelFollower.getConfigurator().apply(ConsShooter.FLYWHEEL_FOLLOWER_CONFIG);

        SparkMaxConfig angleConfig = new SparkMaxConfig();
        angleConfig.closedLoop
            .p(ConsShooter.ANGLE_kP)
            .i(ConsShooter.ANGLE_kI)
            .d(ConsShooter.ANGLE_kD);

        angle.configure(
            angleConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        angleEncoder.setPosition(0.0);
    }

    //Flywheel
    public void setShooterRPM(double rpm) {
        double rps = rpm / 60.0;
        flywheelMain.setControl(flywheelVelocityReq.withVelocity(rps));
        flywheelFollower.setControl(flywheelVelocityReq.withVelocity(rps));
    }

    public void setShooterDuty(double duty) {
        duty = MathUtil.clamp(duty, -1.0, 1.0);
        flywheelMain.setControl(flywheelVoltageReq.withOutput(duty * 12.0));
        flywheelFollower.setControl(flywheelVoltageReq.withOutput(duty * 12.0));
    }

    public void stopShooter() {
        flywheelMain.stopMotor();
        flywheelFollower.stopMotor();
    }

    public double getRPM() {
        return flywheelMain.getVelocity().getValueAsDouble() * 60.0;
    }

    //Intake 
    public void setIntakeTrain(double speed) {
        intakeTrain.set(MathUtil.clamp(speed, -1, 1));
    }

    public void intakeForward() {
        setIntakeTrain(ConsShooter.INTAKE_FORWARD);
    }

    public void intakeReverse() {
        setIntakeTrain(ConsShooter.INTAKE_REVERSE);
    }

    public void stopIntakeTrain() {
        setIntakeTrain(0);
    }

    //Shooter Train
    public void setShooterTrain(double percent) {
        percent = MathUtil.clamp(percent, -1.0, 1.0);
        shooterTrain.setControl(
            shooterTrainVoltageReq.withOutput(percent * 12.0)
        );
    }

    public void shooterTrainForward() {
        setShooterTrain(ConsShooter.SHOOTER_TRAIN_FORWARD);
    }

    public void shooterTrainReverse() {
        setShooterTrain(ConsShooter.SHOOTER_TRAIN_REVERSE);
    }

    public void stopShooterTrain() {
        shooterTrain.stopMotor();
    }

    // uptake
    public void uptakeReverse() {
        shooterTrain.setControl(
            shooterTrainVoltageReq.withOutput(
                ConsShooter.UPTAKE_REVERSE_VOLTAGE
            )
        );
    }

    public void feedForward() {

        setIntakeTrain(ConsShooter.FEED_INTAKE_FORWARD);
        setShooterTrain(ConsShooter.FEED_SHOOTER_FORWARD);
    }

    public void feedReverse() {
        setIntakeTrain(ConsShooter.FEED_INTAKE_REVERSE);
        setShooterTrain(ConsShooter.FEED_SHOOTER_REVERSE);
    }

    public void stopTrain() {
        stopIntakeTrain();
        stopShooterTrain();
    }

    public void setAnglePosition(double pos) {
        anglePID.setReference(pos, ControlType.kPosition);
    }

    public double getAngle() {
        return angleEncoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/RPM", getRPM());
        SmartDashboard.putNumber("Shooter/Angle", getAngle());
    }
}