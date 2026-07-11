package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawDetection;
import frc.robot.LimelightHelpers.RawFiducial;

public class LL4 extends SubsystemBase {

  private final String name;

  private boolean cachedHasTarget = false;
  private double cachedTx = 0.0;
  private double cachedTy = 0.0;
  private double cachedTa = 0.0;
  private double cachedTxnc = 0.0;
  private double cachedTync = 0.0;
  private double cachedTagId = -1.0;
  private double cachedHeartbeat = 0.0;

  private RawFiducial[] cachedRawFiducials = new RawFiducial[0];
  private RawDetection[] cachedRawDetections = new RawDetection[0];

  private int cachedBestFiducialId = -1;
  private double cachedBestTagDistanceMeters = Double.NaN;
  private double cachedDistanceMeters = Double.NaN;

  public LL4() {
    this("limelight-four");
  }

  public LL4(String name) {
    if (name == null || name.isEmpty()) {
      throw new IllegalArgumentException("Limelight name CANNOT be empty");
    }
    this.name = name;
  }

  public boolean hasTarget() {
    return cachedHasTarget;
  }

  public double getTX() {
    return cachedTx;
  }

  public double getTY() {
    return cachedTy;
  }

  public double getTA() {
    return cachedTa;
  }

  public double getTXNC() {
    
    return cachedTxnc;
  }

  public double getTYNC() {
    return cachedTync;
  }

  public double getTagID() {
    return cachedTagId;
  }

  public double getHeartbeat() {
    return cachedHeartbeat;
  }

  public void setPipeline(int index) {
    LimelightHelpers.setPipelineIndex(name, index);
  }

  public void ledPipelineControl() {
    LimelightHelpers.setLEDMode_PipelineControl(name);
  }

  public void ledForceOn() {
    LimelightHelpers.setLEDMode_ForceOn(name);
  }

  public void ledForceOff() {
    LimelightHelpers.setLEDMode_ForceOff(name);
  }

  public void ledForceBlink() {
    LimelightHelpers.setLEDMode_ForceBlink(name);
  }

  public RawFiducial[] getRawFiducials() {
    return cachedRawFiducials;
  }

  public RawDetection[] getRawDetections() {
    return cachedRawDetections;
  }

  public boolean hasTag(int tagId) {
    RawFiducial[] tags = getRawFiducials();
    if (tags == null) return false;

    for (RawFiducial t : tags) {
      if (t != null && t.id == tagId) return true;
    }
    return false;
  }

  public boolean hasSpeakerTag() {
    return hasTag(10) || hasTag(26) || hasTag(21) || hasTag(18)|| hasTag(2)|| hasTag(5);
  }

  public int getBestFiducialId() {
    return cachedBestFiducialId;
  }

  public double getBestGoalYawDeg() {
    if (!cachedHasTarget) return Double.NaN;
    return cachedTx;
  }

  public boolean hasLLTarget() {
    return hasTarget();
  }

  public PoseEstimate getPoseEstimateBlue_MegaTag1() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
  }

  public PoseEstimate getPoseEstimateBlue_MegaTag2() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
  }

  public void setRobotOrientationDeg(
      double yawDeg,
      double yawRateDegPerSec,
      double pitchDeg,
      double pitchRateDegPerSec,
      double rollDeg,
      double rollRateDegPerSec) {
    LimelightHelpers.SetRobotOrientation(
        name,
        yawDeg, yawRateDegPerSec,
        pitchDeg, pitchRateDegPerSec,
        rollDeg, rollRateDegPerSec);
  }

  public LimelightHelpers.IMUData getIMU() {
    return LimelightHelpers.getIMUData(name);
  }

  public void setIMUMode(int mode) {
    LimelightHelpers.SetIMUMode(name, mode);
  }

  public void setIMUAssistAlpha(double alpha) {
    LimelightHelpers.SetIMUAssistAlpha(name, alpha);
  }

  public double getBestTagDistanceMeters() {
    return cachedBestTagDistanceMeters;
  }

  public double getDistanceMeters() {
    return cachedDistanceMeters;
  }

  @Override
  public void periodic() {

    cachedHeartbeat = LimelightHelpers.getHeartbeat(name);
    cachedHasTarget = LimelightHelpers.getTV(name);
    cachedTx = LimelightHelpers.getTX(name);
    cachedTy = LimelightHelpers.getTY(name);
    cachedTa = LimelightHelpers.getTA(name);
    cachedTxnc = LimelightHelpers.getTXNC(name);
    cachedTync = LimelightHelpers.getTYNC(name);
    cachedTagId = LimelightHelpers.getFiducialID(name);

    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(name);
    cachedRawFiducials = (fiducials != null) ? fiducials : new RawFiducial[0];

    RawDetection[] detections = LimelightHelpers.getRawDetections(name);
    cachedRawDetections = (detections != null) ? detections : new RawDetection[0];

    cachedBestFiducialId = -1;
    cachedBestTagDistanceMeters = Double.NaN;
    cachedDistanceMeters = Double.NaN;

    var results = LimelightHelpers.getLatestResults(name);
    if (results != null && results.targets_Fiducials != null) {
      LimelightHelpers.LimelightTarget_Fiducial best = null;

      for (var t : results.targets_Fiducials) {
        if (t == null) continue;
        if (best == null || t.ta > best.ta) best = t;
      }

      if (best != null) {
        cachedBestFiducialId = (int) best.fiducialID;

        Pose3d tagInCam = best.getTargetPose_CameraSpace();
        if (tagInCam != null) {
          cachedBestTagDistanceMeters = tagInCam.getTranslation().getNorm();

          double forward_z = -0.6;
          double right_x = 0.0;
          double up_y = -0.7;

          Transform3d tagToPOI = new Transform3d(
              new Translation3d(right_x, up_y, forward_z),
              new Rotation3d());

          Pose3d poiInCam = tagInCam.transformBy(tagToPOI);
          cachedDistanceMeters = poiInCam.getTranslation().getNorm();
        }
      }
    }

    SmartDashboard.putBoolean("LL4/HasTarget", cachedHasTarget);
    SmartDashboard.putNumber("LL4/tx", cachedTx);
    SmartDashboard.putNumber("LL4/tagID", cachedTagId);
    SmartDashboard.putNumber("LL4/Meters", getDistanceMeters());

  }
}
// package frc.robot.subsystems;

// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.Vector;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.robot.LimelightHelpers;
// import frc.robot.LimelightHelpers.PoseEstimate;

// public class LL4 extends SubsystemBase {

//     private final String name;

//     private boolean cachedHasTarget = false;
//     private double cachedTx = 0.0;
//     private double cachedTy = 0.0;
//     private double cachedTagId = -1.0;
//     private double cachedDistanceMeters = Double.NaN;

//     public LL4() {
//         this("limelight-four");
//     }

//     public LL4(String name) {
//         this.name = name;
//     }

//     public void updateRobotOrientation(double yawDeg, double yawRate) {
//         LimelightHelpers.SetRobotOrientation(name, yawDeg, yawRate, 0, 0, 0, 0);
//     }


//     public PoseEstimate getPoseEstimate() {
//         return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
//     }

//     public Vector<N3> getEstimationStdDevs(PoseEstimate estimate) {
//         double xyStdDev = 0.4;
//         if (estimate.tagCount >= 2) {
//             xyStdDev = 0.15;
//         } else if (estimate.avgTagDist > 4.0) {
//             xyStdDev = 1.2;
//         }
//         return VecBuilder.fill(xyStdDev, xyStdDev, 9999999.0);
//     }

//     public boolean hasTarget() { return cachedHasTarget; }
//     public double getTX() { return cachedTx; }
//     public double getTY() { return cachedTy; }
//     public double getDistanceMeters() { return cachedDistanceMeters; }
    
   
//     public double getID() { return cachedTagId; }

//     @Override
//     public void periodic() {

//         cachedHasTarget = LimelightHelpers.getTV(name);
//         cachedTx = LimelightHelpers.getTX(name);
//         cachedTy = LimelightHelpers.getTY(name);
//         cachedTagId = LimelightHelpers.getFiducialID(name);

//         // 2. 處理 Fiducial (POI 距離計算)
//         var results = LimelightHelpers.getLatestResults(name);

//         // 注意：這裡只影響 cachedDistanceMeters，不影響 getPoseEstimate() 的全場定位
//         if (results != null && results.targets_Fiducials != null && results.targets_Fiducials.length > 0) {

//             LimelightHelpers.LimelightTarget_Fiducial best = null;

//             for (LimelightHelpers.LimelightTarget_Fiducial t : results.targets_Fiducials) {
//                 // 如果你想讓距離顯示也只針對 10/26，可以加判斷，
//                 // 但建議保持「選取面積最大者」即可，因為過濾邏輯會寫在 Command 裡。
//                 if (best == null || t.ta > best.ta) {
//                     best = t;
//                 }
//             }

//             if (best != null) {
//                 Pose3d tagInCam = best.getTargetPose_CameraSpace();
//                 if (tagInCam != null) {
//                     // 2026 POI 補償 (假設值)
//                     Transform3d tagToPOI = new Transform3d(
//                         new Translation3d(0.0, -0.7, -0.6),
//                         new Rotation3d()
//                     );

//                     Pose3d poiInCam = tagInCam.transformBy(tagToPOI);
//                     cachedDistanceMeters = poiInCam.getTranslation().getNorm();
//                 }
//             }
//         } else {
//             cachedDistanceMeters = Double.NaN;
//         }

//         // 3. Dashboard 更新
//         SmartDashboard.putBoolean("LL4/HasTarget", cachedHasTarget);
//         SmartDashboard.putNumber("LL4/Current ID", cachedTagId);
//         SmartDashboard.putNumber("LL4/POI Distance", cachedDistanceMeters);

//         if (cachedHasTarget) {
//             var pose = getPoseEstimate();
//             if (pose != null) {
//                 // 這裡的 pose 是基於所有可見 Tag 計算的，不會被限制
//                 SmartDashboard.putNumber("LL4/MegaTag2 X", pose.pose.getX());
//                 SmartDashboard.putNumber("LL4/MegaTag2 Y", pose.pose.getY());
//                 SmartDashboard.putNumber("LL4/TagCount", pose.tagCount);
//             }
//         }
//     }
// }