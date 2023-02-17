// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.commands.ChangeStream;
import com.adambots.utils.LimelightHelpers;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

 public class VisionSubsystem extends SubsystemBase {
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    public VisionSubsystem() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
        initialize();
       // table.getEntry("ledMode").setNumber(2);
    }

    public void initialize() {
        SmartDashboard.putData("change-stream", new ChangeStream());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("distance-away", table.getEntry("tx").getDouble(0));
        SmartDashboard.putNumber("detected", table.getEntry("tv").getInteger(0));
        SmartDashboard.putNumber("turn-angle", LimelightHelpers.getLatestResults("limelight").targetingResults.getBotPose2d_wpiBlue().getRotation().getDegrees());
       // SmartDashboard.putData(null);
    }
}