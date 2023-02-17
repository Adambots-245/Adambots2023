package com.adambots.utils;

import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionHelpers {
    public VisionHelpers() {
    }

    public double getDistanceAway() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    }

    public double getDetected() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getInteger(0);
    }

    public double getTurnAngle() {
        return LimelightHelpers.getLatestResults("limelight").targetingResults.getBotPose2d_wpiBlue()
                .getRotation().getDegrees();
    }
}