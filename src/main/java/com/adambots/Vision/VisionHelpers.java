package com.adambots.Vision;

import com.adambots.Constants.VisionConstants;
import com.adambots.utils.LimelightHelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class VisionHelpers {
    private VisionHelpers() {
    }

    public static double getDistanceAway() {
        // return
        // NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        return LimelightHelpers.getTX(VisionConstants.Limelight1);
    }

    public static boolean isDetected() {
        // return
        // NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getInteger(0);
        return LimelightHelpers.getTV(VisionConstants.Limelight1);
    }

    public static double getTurnAngle() {
        // return LimelightHelpers.getLatestResults("limelight").targetingResults.getBotPose2d_wpiBlue().getRotation()
        //         .getDegrees();
        return 0.0;
    }

    public static double getLeftRightDistance() {
        // return
        // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getInteger(0);
        return LimelightHelpers.getTY(VisionConstants.Limelight1);
    }

    public static double getDetectedResult() {
        // return
        // NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(0);
        return LimelightHelpers.getFiducialID(VisionConstants.Limelight1);
    }

    public static Field2d getAprilTagField2d() {
        // return VisionProcessingSubsystem.getAprilField2d();
        Field2d field = new Field2d();

        // field.setRobotPose(
        //         LimelightHelpers.getLatestResults(VisionConstants.Limelight1).targetingResults.getBotPose2d_wpiBlue());
        // return field;
        return null;

    }

    public static Pose2d getAprilTagPose2d() {
        return LimelightHelpers.getLatestResults(VisionConstants.Limelight1).targetingResults.getBotPose2d_wpiBlue();
        // return null;

    }

    public static double[] getAprilTagArrayPos() {
        // double [] def = {0,0,0,0,0};
        // return
        // NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(def);
        return LimelightHelpers.getLimelightNTDoubleArray(VisionConstants.Limelight1, "botpose");
    }
}