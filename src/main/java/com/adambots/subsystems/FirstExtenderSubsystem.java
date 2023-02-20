// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants;
import com.adambots.sensors.PhotoEye;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FirstExtenderSubsystem extends SubsystemBase {

  private final TalonFX firstExtender;
  private final PIDController pid;
  private final PhotoEye photoEye;

  private double firstExtenderSpeed = 0;
  private double targetPosition = Constants.GrabbyConstants.initState.getFirstExtendTarget();

  public FirstExtenderSubsystem(TalonFX firstExtender, PhotoEye photoEye) {
    this.firstExtender = firstExtender;
    firstExtender.setInverted(true);
    this.photoEye = photoEye;
    pid = new PIDController(Constants.GrabbyConstants.firstExtenderP, Constants.GrabbyConstants.firstExtenderI, Constants.GrabbyConstants.firstExtenderD);
  }

  public void changeTarget(double newTarget){
    targetPosition = newTarget;
    pid.reset();
  }

  public void fullOut(){
    targetPosition = Constants.GrabbyConstants.highConeState.getFirstExtendTarget();
    pid.reset();
  }

  public void fullIn(){
    targetPosition = 0;
    pid.reset();
  }

  public void manualOut(){
    targetPosition = firstExtender.getSelectedSensorPosition() + 2000;
  }

  public void manualIn(){
    targetPosition = firstExtender.getSelectedSensorPosition() - 2000;
  }

  public void stopExtending(){
    targetPosition = firstExtender.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("First Extender Encoder", firstExtender.getSelectedSensorPosition());

    if(targetPosition > 0){
      firstExtenderSpeed = pid.calculate(firstExtender.getSelectedSensorPosition(), targetPosition);
      firstExtenderSpeed = MathUtil.clamp(firstExtenderSpeed, -Constants.GrabbyConstants.extenderSpeed, Constants.GrabbyConstants.extenderSpeed);
    }else if(!photoEye.isDetecting()){
      firstExtenderSpeed = -Constants.GrabbyConstants.extenderSpeed;
    }
    
    failsafes();
    firstExtender.set(ControlMode.PercentOutput, firstExtenderSpeed);
    SmartDashboard.putNumber("First Extender Speed", firstExtenderSpeed);
  }

  private void failsafes() {
    if(firstExtender.getSelectedSensorPosition() >= Constants.GrabbyConstants.highConeState.getFirstExtendTarget() && firstExtenderSpeed > 0){
      firstExtenderSpeed = 0;
    }

    if(photoEye.isDetecting()){
      firstExtender.setSelectedSensorPosition(0);
      if(firstExtenderSpeed < 0){
        firstExtenderSpeed = 0;
      }
    }
  }
}
