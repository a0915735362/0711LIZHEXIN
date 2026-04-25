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

    private final TalonFX flywheelMain = new TalonFX(ConsShooter.FLYWHEEL_MAIN_ID,ConsIntake.KcanBusName);
    private final TalonFX flywheelFollower = new TalonFX(ConsShooter.FLYWHEEL_FOLLOWER_ID,ConsIntake.KcanBusName);

    private final SparkMax intakeTrain = new SparkMax(ConsShooter. INTAKE_TRAIN_ID, MotorType.kBrushless);
    private final SparkMax shooterTrain = new SparkMax(ConsShooter.SHOOTER_TRAIN_ID, MotorType.kBrushless);

    private final SparkMax angle = new SparkMax(ConsShooter.ANGLE_ID, MotorType.kBrushless);
    private final RelativeEncoder angleEncoder = angle.getEncoder();
    private final SparkClosedLoopController anglePID = angle.getClosedLoopController();

    private final VoltageOut flywheelVoltageReq = new VoltageOut(0);
    private final VelocityVoltage flywheelVelocityReq = new VelocityVoltage(0).withSlot(0);

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

    public void setIntakeTrain(double speed) {
        intakeTrain.set(MathUtil.clamp(speed, -1, 1));
    }
//把球吸進來
    public void intakeForward() {
        setIntakeTrain(-0.6);
    }
//吐出來j
    public void intakeReverse() {
        setIntakeTrain(0.75);
    }

    public void stopIntakeTrain() {
        setIntakeTrain(0);
    }

    public void setShooterTrain(double speed) {
        shooterTrain.set(MathUtil.clamp(speed, -1, 1));
    }

    public void shooterTrainForward() {
        setShooterTrain(-0.85);
    }

    public void shooterTrainReverse() {
        setShooterTrain(0.7);
    }

    public void stopShooterTrain() {
        setShooterTrain(0);
    }

    public void feedForward() {
        setIntakeTrain(0.6);
        setShooterTrain(0.7);
    }

    public void feedReverse() {
        setIntakeTrain(-0.7);
        setShooterTrain(-0.8);
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