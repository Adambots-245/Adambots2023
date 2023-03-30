package com.adambots.Vision;

import com.adambots.Constants.VisionConstants;
import com.adambots.utils.LimelightHelpers;
import com.adambots.utils.LimelightHelpers.LimelightTarget_Detector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class VisionHelpers {

    private VisionHelpers() {
        // SmartDashboard.putBoolean("isDetectingPieces()", isDetectingPieces());
    }

    public static double getTX() {
        // return
        // NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        return LimelightHelpers.getTX(VisionConstants.Limelight1);
    }
    public static boolean isAligned() {
        // return
        // NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        return isDetected() && Math.abs(LimelightHelpers.getTX(VisionConstants.Limelight1)) <= 1;
    }

    public static boolean isDetected() {
        // return
        // NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getInteger(0);
        return LimelightHelpers.getTV(VisionConstants.Limelight1);
    }

    public static double getTurnAngle() {
        // return
        // LimelightHelpers.getLatestResults("limelight").targetingResults.getBotPose2d_wpiBlue().getRotation()
        // .getDegrees();
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
        // LimelightHelpers.getLatestResults(VisionConstants.Limelight1).targetingResults.getBotPose2d_wpiBlue());
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

    public static String getClassName() {
        LimelightTarget_Detector[] targetDetector = LimelightHelpers.getLatestResults(VisionConstants.Limelight1).targetingResults.targets_Detector;

        if (targetDetector == null) {
            return "";
        }

        if (targetDetector.length == 0) {
            return "";
        }

        return targetDetector[0].className;
    }

    public static boolean isDetectingPieces() {

        if (getClassName() == "") {
            return false;
        } else {
            return true;
        }
    }

    public static boolean isDetectingPieces(String type){
        LimelightTarget_Detector[] targetDetector = LimelightHelpers
                .getLatestResults(VisionConstants.Limelight1).targetingResults.targets_Detector;
        
        int index = -1;
        if(targetDetector != null){
         for(int i = targetDetector.length - 1; i >= 0; i--){
            if(targetDetector[i].className.equals(type)){
                index = i;
            }
         }
        }

        return index != -1;
    }

    public static double getPieceX() {
        LimelightTarget_Detector[] targetDetector = LimelightHelpers
                .getLatestResults(VisionConstants.Limelight1).targetingResults.targets_Detector;

        if (targetDetector == null) {
            return -1.0;
        }

        if (targetDetector.length == 0) {
            return -1.0;
        }

        return targetDetector[0].tx;
    }

    public static double getPieceX(String type) {
        LimelightTarget_Detector[] targetDetector = LimelightHelpers
                .getLatestResults(VisionConstants.Limelight1).targetingResults.targets_Detector;

        int index = -1;
        if(targetDetector != null){
         for(int i = targetDetector.length - 1; i >= 0; i--){
            if(targetDetector[i].className.equals(type)){
                index = i;
            }
         }
        }

        if(index == -1){
            return -1.0;
        }

        return targetDetector[index].tx;
    }

    public static double getPieceY() {
        LimelightTarget_Detector[] targetDetector = LimelightHelpers
                .getLatestResults(VisionConstants.Limelight1).targetingResults.targets_Detector;

        if (targetDetector == null) {
            return -1.0;
        }

        if (targetDetector.length == 0) {
            return -1.0;
        }

        return targetDetector[0].ty;
    }

    public static double getPieceY(String type){
        LimelightTarget_Detector[] targetDetector = LimelightHelpers
                .getLatestResults(VisionConstants.Limelight1).targetingResults.targets_Detector;

        int index = -1;
        if(targetDetector != null){
         for(int i = targetDetector.length - 1; i >= 0; i--){
            if(targetDetector[i].className.equals(type)){
                index = i;
            }
         }
        }

        if(index == -1){
            return -1.0;
        }

        return targetDetector[index].ty;
    }

    public static double getDistanceToObject() {
        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 0.1; //25.0

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 4;

        // distance from the target to the floor
        double goalHeightInches = 4;

        double angleToGoalDegrees = limelightMountAngleDegrees + getPieceY();
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        // calculate distance
        return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }

    public static void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(VisionConstants.Limelight1, pipeline); 
    }
}
