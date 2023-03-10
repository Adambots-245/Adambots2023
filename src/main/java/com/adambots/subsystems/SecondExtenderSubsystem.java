// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants;
import com.adambots.Constants.GrabbyConstants;
import com.adambots.sensors.PhotoEye;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SecondExtenderSubsystem extends SubsystemBase {
  
  private final TalonFX secondExtender;
  private final PIDController pid;
  private final PhotoEye photoEye;
  private final WPI_CANCoder armLifterEncoder;


  private double secondExtenderSpeed = 0;
  private double targetPosition = Constants.GrabbyConstants.initState.getSecondExtendTarget();

  public SecondExtenderSubsystem(TalonFX secondExtender, PhotoEye photoEye, WPI_CANCoder armLifterEncoder) {
    this.secondExtender = secondExtender;
    secondExtender.setInverted(true);
    this.photoEye = photoEye;
    pid = new PIDController(Constants.GrabbyConstants.secondExtenderP, Constants.GrabbyConstants.secondExtenderI, Constants.GrabbyConstants.secondExtenderD);
    this.armLifterEncoder = armLifterEncoder;
    // secondExtender.configOpenloopRamp(0.2);
  }

  public void changeTarget(double newTarget){
    targetPosition = newTarget;
    pid.reset();
  }

  public void fullOut(){
    targetPosition = Constants.GrabbyConstants.highConeState.getSecondExtendTarget();
    pid.reset();
  }

  public void fullIn(){
    targetPosition = 0;
    pid.reset();
  }

  public void manualOut(){
    targetPosition = 99999999;
  }

  public void manualIn(){
    targetPosition = -99999999;
  }

  public void stopExtending(){
    targetPosition = secondExtender.getSelectedSensorPosition();
  }

  public boolean isMaxRetracted () {
    return photoEye.isDetecting();
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Second Extender Encoder", secondExtender.getSelectedSensorPosition()/GrabbyConstants.armEncoderCPR);
    
    //Horizontal and vertical expansion limits
    // double horizontalLimit = Math.abs(GrabbyConstants.horizontalMaxEncoderValue/Math.cos(Math.toRadians(armLifterEncoder.getAbsolutePosition()+GrabbyConstants.limitOffset)));
    // double verticalLimit = Math.abs(GrabbyConstants.veritcalMaxEncoderValue/Math.sin(Math.toRadians(armLifterEncoder.getAbsolutePosition()+GrabbyConstants.limitOffset)));

    // horizontalLimit = MathUtil.clamp(horizontalLimit, GrabbyConstants.horizontalMaxEncoderValue, GrabbyConstants.horizontalMaxEncoderValue*3);
    // verticalLimit = MathUtil.clamp(verticalLimit, GrabbyConstants.veritcalMaxEncoderValue, GrabbyConstants.veritcalMaxEncoderValue*3);

    SmartDashboard.putNumber("Arm Encoder W/ Offset", armLifterEncoder.getAbsolutePosition()+GrabbyConstants.limitOffset);
    // SmartDashboard.putNumber("horizontalLimit", horizontalLimit);
    // SmartDashboard.putNumber("verticalLimit", verticalLimit);  

    if(targetPosition > 0){
      secondExtenderSpeed = pid.calculate(secondExtender.getSelectedSensorPosition(), targetPosition);
      secondExtenderSpeed = MathUtil.clamp(secondExtenderSpeed, -Constants.GrabbyConstants.extenderSpeed, Constants.GrabbyConstants.extenderSpeed);
    }else if(!isMaxRetracted()){
      secondExtenderSpeed = -Constants.GrabbyConstants.extenderSpeed;
    }

    failsafes();
    secondExtender.set(ControlMode.PercentOutput, secondExtenderSpeed);
    // secondExtender.set(ControlMode.PercentOutput, 0);

    SmartDashboard.putNumber("Second Extender Speed", secondExtenderSpeed);
    SmartDashboard.putBoolean("decond Photo Eye", photoEye.isDetecting());
  }

  private void failsafes() {
    //Preventing the arm from going too far out or in

    if(secondExtender.getSelectedSensorPosition() > GrabbyConstants.horizontalMaxEncoderValue && armLifterEncoder.getAbsolutePosition()+GrabbyConstants.limitOffset < 5 && secondExtenderSpeed > 0){
      secondExtenderSpeed = 0;
    }

    if(secondExtender.getSelectedSensorPosition() > GrabbyConstants.veritcalMaxEncoderValue && armLifterEncoder.getAbsolutePosition() > 200 && secondExtenderSpeed > 0){
      secondExtenderSpeed = 0;
    }

    if(secondExtender.getSelectedSensorPosition() >= Constants.GrabbyConstants.secondExtenderMaxExtend && secondExtenderSpeed > 0){
      secondExtenderSpeed = 0;
    }

    if(isMaxRetracted()){
      secondExtender.setSelectedSensorPosition(0);
      if(secondExtenderSpeed < 0){
        secondExtenderSpeed = 0;
      }
    }

    //Preventing the arm from extending into the ground
    if (armLifterEncoder.getAbsolutePosition() <= Constants.GrabbyConstants.groundLifterValue + 10) {
        targetPosition = 0;
    } 
  }
}
