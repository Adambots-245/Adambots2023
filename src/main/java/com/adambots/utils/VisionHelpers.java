package com.adambots.utils;

import com.adambots.subsystems.VisionProcessingSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class VisionHelpers {
    public VisionHelpers() {
    }

    public static double getDistanceAway() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    }

    public static double getDetected() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getInteger(0);
    }

    public static double getTurnAngle() {
        return LimelightHelpers.getLatestResults("limelight").targetingResults.getBotPose2d_wpiBlue().getRotation()
                .getDegrees();
    }

    public static double getLeftRightDistance() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getInteger(0);
    }

    public static double getDetectedResult() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tclass").getInteger(0);
    }

    public static double getPose2d() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tclass").getInteger(0);
    }

    public static Field2d getAprilTagField2d() {
        return VisionProcessingSubsystem.getAprilField2d();
    }

    public static Pose2d getAprilTagPose2d() {
        return LimelightHelpers.getLatestResults("limelight").targetingResults.getBotPose2d_wpiBlue();
    }

    public static double[] getAprilTagArrayPos() {
        double [] def = {0,0,0,0,0};
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(def);
    }
}