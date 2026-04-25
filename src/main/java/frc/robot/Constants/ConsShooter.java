// package frc.robot.constants;

// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// public final class ConsShooter {

//     public static final int FLYWHEEL_MAIN_ID = 42;        
//     public static final int FLYWHEEL_FOLLOWER_ID = 40;    

//     public static final int INTAKE_TRAIN_ID = 4;          
//     public static final int SHOOTER_TRAIN_ID = 5;      
//     public static final int ANGLE_ID = 6;               
//     public static final boolean FLYWHEEL_FOLLOWER_OPPOSE_MAIN = true;

//     public static final InvertedValue FLYWHEEL_MAIN_INVERT =
//         InvertedValue.CounterClockwise_Positive;

//     public static final double FLYWHEEL_kP = 0.35;
//     public static final double FLYWHEEL_kI = 0.0;
//     public static final double FLYWHEEL_kD = 0.0;
//     public static final double FLYWHEEL_kS = 0.0;
//     public static final double FLYWHEEL_kV = 0.0;
//     public static final double FLYWHEEL_kA = 0.0;

//     public static final double FLYWHEEL_FORWARD_VOLTS = 8.0;
//     public static final double FLYWHEEL_REVERSE_VOLTS = -8.0;
//     public static final double FLYWHEEL_STOP_VOLTS = 0.0;



//     public static final double ANGLE_kP = 0.6;
//     public static final double ANGLE_kI = 0.0;
//     public static final double ANGLE_kD = 0.0;


//     public static final TalonFXConfiguration FLYWHEEL_MAIN_CONFIG = new TalonFXConfiguration();
//     public static final TalonFXConfiguration FLYWHEEL_FOLLOWER_CONFIG = new TalonFXConfiguration();

//     private static final Slot0Configs FLYWHEEL_SLOT0 =
//         new Slot0Configs()
//             .withKP(FLYWHEEL_kP)
//             .withKI(FLYWHEEL_kI)
//             .withKD(FLYWHEEL_kD)
//             .withKS(FLYWHEEL_kS)
//             .withKV(FLYWHEEL_kV)
//             .withKA(FLYWHEEL_kA);

//     static {

//         FLYWHEEL_MAIN_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
//         FLYWHEEL_MAIN_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
//         FLYWHEEL_MAIN_CONFIG.Slot0 = FLYWHEEL_SLOT0;

//         FLYWHEEL_FOLLOWER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
//         FLYWHEEL_FOLLOWER_CONFIG.MotorOutput.Inverted = FLYWHEEL_MAIN_INVERT;
//         FLYWHEEL_FOLLOWER_CONFIG.Slot0 = FLYWHEEL_SLOT0;
//     }

//     private ConsShooter() {
//     }
// }
package frc.robot.Constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class ConsShooter {


    public static final int FLYWHEEL_MAIN_ID = 42;
    public static final int FLYWHEEL_FOLLOWER_ID = 40;

    public static final int INTAKE_TRAIN_ID = 4;
    public static final int SHOOTER_TRAIN_ID = 5;
    public static final int ANGLE_ID = 6;

    public static final boolean FLYWHEEL_FOLLOWER_OPPOSE_MAIN = true;

    public static final InvertedValue FLYWHEEL_MAIN_INVERT =
        InvertedValue.CounterClockwise_Positive;

    public static final double FLYWHEEL_kP = 0.35;
    public static final double FLYWHEEL_kI = 0.0;
    public static final double FLYWHEEL_kD = 0.0;
    public static final double FLYWHEEL_kS = 0.0;
    public static final double FLYWHEEL_kV = 0.0;
    public static final double FLYWHEEL_kA = 0.0;

    public static final double FLYWHEEL_FORWARD_VOLTS = 8.0;
    public static final double FLYWHEEL_REVERSE_VOLTS = -8.0;
    public static final double FLYWHEEL_STOP_VOLTS = 0.0;

 
    public static final double ANGLE_kP = 0.175;
    public static final double ANGLE_kS = 0.0;
    public static final double ANGLE_kI=0;
    public static final double ANGLE_kD=0;  
    public static final double ANGLE_kG = 00;

    public static final double ANGLE_TOLERANCE = 0.005;

    
    public static final TalonFXConfiguration FLYWHEEL_MAIN_CONFIG =
        new TalonFXConfiguration();

    public static final TalonFXConfiguration FLYWHEEL_FOLLOWER_CONFIG =
        new TalonFXConfiguration();

    private static final Slot0Configs FLYWHEEL_SLOT0 =
        new Slot0Configs()
            .withKP(FLYWHEEL_kP)
            .withKI(FLYWHEEL_kI)
            .withKD(FLYWHEEL_kD)
            .withKS(FLYWHEEL_kS)
            .withKV(FLYWHEEL_kV)
            .withKA(FLYWHEEL_kA);

    static {

        FLYWHEEL_MAIN_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        FLYWHEEL_MAIN_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        FLYWHEEL_MAIN_CONFIG.Slot0 = FLYWHEEL_SLOT0;

        FLYWHEEL_FOLLOWER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        FLYWHEEL_FOLLOWER_CONFIG.MotorOutput.Inverted = FLYWHEEL_MAIN_INVERT;
        FLYWHEEL_FOLLOWER_CONFIG.Slot0 = FLYWHEEL_SLOT0;
    }

    private ConsShooter() {}
}