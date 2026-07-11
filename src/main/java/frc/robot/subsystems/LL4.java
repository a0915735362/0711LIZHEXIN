package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers.RawFiducial;

public class LL4 extends SubsystemBase {

    private final String name;
    private final Field2d m_field = new Field2d();
    
    private static final Transform3d TAG_TO_TARGET_POI = new Transform3d(
        new Translation3d(0.0, 0.0, 0.0),
        new Rotation3d()
    );

    private static final Pose3d SHOOTER_POSE_IN_CAM = new Pose3d(
        new Translation3d(0.0, 0.2, 0.0),
        new Rotation3d()
    );

    private boolean cachedHasTarget = false;
    private double cachedTx = 0.0;
    private double cachedTagId = -1.0;
    public double cachedDistanceMeters = Double.NaN;
    private RawFiducial[] cachedRawFiducials = new RawFiducial[0];

    public LL4() {
        this("limelight-four");
    }

    public LL4(String name) {
        this.name = name;
        SmartDashboard.putData("LL4/Field", m_field);
    }

    public boolean hasTarget() {
        return cachedHasTarget;
    }

    public double getTX() {
        return cachedTx;
    }

    public double getDistanceMeters() {
        return cachedDistanceMeters;
    }

    @Override
    public void periodic() {
        Pose2d botPose = LimelightHelpers.getBotPose2d_wpiBlue(name);
        m_field.setRobotPose(botPose);

        LimelightResults results = LimelightHelpers.getLatestResults(name);

        if (results != null && results.targets_Fiducials != null && results.targets_Fiducials.length > 0) {
            cachedHasTarget = true;
            LimelightTarget_Fiducial bestTarget = results.targets_Fiducials[0];

            cachedTx = bestTarget.tx;
            cachedTagId = bestTarget.fiducialID;
            cachedRawFiducials = LimelightHelpers.getRawFiducials(name);

            Pose3d tagInCam = bestTarget.getTargetPose_CameraSpace();

            if (tagInCam != null) {
                Pose3d poiInCam = tagInCam.transformBy(TAG_TO_TARGET_POI);
                Pose3d poiInShooter = poiInCam.relativeTo(SHOOTER_POSE_IN_CAM);
                cachedDistanceMeters = poiInShooter.getTranslation().getNorm();
            }
        } else {
            cachedHasTarget = false;
            cachedDistanceMeters = Double.NaN;
        }

        updateDashboard();
    }

    private void updateDashboard() {
        SmartDashboard.putBoolean("LL4/看到目標", cachedHasTarget);
        SmartDashboard.putNumber("LL4/目標ID", cachedTagId);
        SmartDashboard.putNumber("LL4/發射器實際距離_M", cachedDistanceMeters);
        SmartDashboard.putNumber("LL4/橫向偏移_TX", cachedTx);
    }
}