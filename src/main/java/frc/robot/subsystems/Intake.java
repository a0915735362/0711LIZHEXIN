// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.RelativeEncoder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkBase.PersistMode;

// import frc.robot.Constants.ConsIntake;

// public class Intake extends SubsystemBase {

//     private final SparkMax rollerMotor = new SparkMax(ConsIntake.ROLLER_MOTOR_ID, MotorType.kBrushless);
//     private final SparkMax rollerMotor2 = new SparkMax(ConsIntake.ROLLER_FOLLOWER_ID, MotorType.kBrushless);

//     private final SparkMax intakeLeft = new SparkMax(ConsIntake.INTAKE_LEFT_ID, MotorType.kBrushless);
//     private final SparkMax intakeRight = new SparkMax(ConsIntake.INTAKE_RIGHT_ID, MotorType.kBrushless);

//     private final RelativeEncoder leftEncoder = intakeLeft.getEncoder();
//     private final RelativeEncoder rightEncoder = intakeRight.getEncoder();

//     public Intake() {
//         leftEncoder.setPosition(0);
//         rightEncoder.setPosition(0);

//         SparkMaxConfig followerConfig = new SparkMaxConfig();
//         followerConfig.follow(rollerMotor, true);

//         rollerMotor2.configure(
//             followerConfig,
//             ResetMode.kNoResetSafeParameters,
//             PersistMode.kPersistParameters
//         );
//     }

//     @Override
//     public void periodic() {
//         SmartDashboard.putNumber("Intake/Left Encoder", leftEncoder.getPosition());
//         SmartDashboard.putNumber("Intake/Right Encoder", rightEncoder.getPosition());

//         SmartDashboard.putNumber("Intake/Left Amps", intakeLeft.getOutputCurrent());
//         SmartDashboard.putNumber("Intake/Right Amps", intakeRight.getOutputCurrent());
//         SmartDashboard.putNumber("Intake/Roller Amps", rollerMotor.getOutputCurrent());
//     }

//     public void extend() {
//         intakeLeft.set(ConsIntake.LEFT_EXTEND_SPEED);
//         intakeRight.set(-ConsIntake.RIGHT_EXTEND_SPEED);
//     }

//     public void retract() {
//         intakeLeft.set(-ConsIntake.LEFT_BACK_SPEED);
//         intakeRight.set(ConsIntake.RIGHT_BACK_SPEED);
//     }

//     public void stopArm() {
//         intakeLeft.set(0);
//         intakeRight.set(0);
//     }


//     public void runRoller() {
//         rollerMotor.set(ConsIntake.ROLLER_SPEED); 
//     }

//     public void intake() {
//         // runRoller(Math.abs(ConsIntake.ROLLER_SPEED));
//     }

//     public void outtake() {
//         double speed = SmartDashboard.getNumber(
//             "Outtake Speed",
//             Math.abs(ConsIntake.OUTTAKE_SPEED)
//         );
//         // runRoller(-Math.abs(speed));
//     }

//     public void stopRoller() {
//         rollerMotor.set(0);
//     }

// }
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ConsIntake;

public class Intake extends SubsystemBase {

    private final TalonFX rollerRightMotor =
        new TalonFX(ConsIntake.ROLLER_RIGHT_MOTOR_ID);

    private final TalonFX rollerLeftMotor =
        new TalonFX(ConsIntake.ROLLER_LEFT_MOTOR_ID);

    private final VoltageOut rightOutput = new VoltageOut(0);
    private final VoltageOut leftOutput = new VoltageOut(0);


    private final SparkMax intakeLeft =
        new SparkMax(ConsIntake.INTAKE_LEFT_ID, MotorType.kBrushless);

    private final SparkMax intakeRight =
        new SparkMax(ConsIntake.INTAKE_RIGHT_ID, MotorType.kBrushless);


    public Intake() {

        MotorOutputConfigs leftConfig = new MotorOutputConfigs();
        leftConfig.Inverted = InvertedValue.Clockwise_Positive;
        rollerLeftMotor.getConfigurator().apply(leftConfig);

        MotorOutputConfigs rightConfig = new MotorOutputConfigs();
        rightConfig.Inverted = InvertedValue.CounterClockwise_Positive;
        rollerRightMotor.getConfigurator().apply(rightConfig);

        SparkMaxConfig leftSpark = new SparkMaxConfig();
        leftSpark.inverted(true);

        SparkMaxConfig rightSpark = new SparkMaxConfig();
        rightSpark.inverted(false);

        intakeLeft.configure(leftSpark,
                SparkMax.ResetMode.kResetSafeParameters,
                SparkMax.PersistMode.kNoPersistParameters);

        intakeRight.configure(rightSpark,
                SparkMax.ResetMode.kResetSafeParameters,
                SparkMax.PersistMode.kNoPersistParameters);
    }


    private void setRollerVoltage(double voltage) {
        rollerRightMotor.setControl(rightOutput.withOutput(voltage));
        rollerLeftMotor.setControl(leftOutput.withOutput(voltage));
    }

    private void setArmSpeed(double speed) {
        intakeLeft.set(speed);
        intakeRight.set(speed);
    }

    public void intake() {
        setRollerVoltage(ConsIntake.ROLLER_VOLTAGE);
    }

    public void outtake() {
        setRollerVoltage(-ConsIntake.ROLLER_VOLTAGE);
    }

    public void stopRoller() {
        setRollerVoltage(0);
    }

    public void extend() {
        setArmSpeed(ConsIntake.EXTEND_SPEED);
    }

    public void retract() {
        setArmSpeed(ConsIntake.BACK_SPEED);
    }

    public void stopArm() {
        setArmSpeed(0);
    }
}