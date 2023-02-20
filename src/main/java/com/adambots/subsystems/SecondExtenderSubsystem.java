// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants;
import com.adambots.sensors.PhotoEye;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SecondExtenderSubsystem extends SubsystemBase {
  
  private final TalonFX secondExtender;
  private final PIDController pid;
  private final PhotoEye photoEye;

  private double secondExtenderSpeed = 0;
  private double targetPosition = Constants.GrabbyConstants.initState.getSecondExtendTarget();

  public SecondExtenderSubsystem(TalonFX secondExtender, PhotoEye photoEye) {
    this.secondExtender = secondExtender;
    this.photoEye = photoEye;
    pid = new PIDController(Constants.GrabbyConstants.secondExtenderP, Constants.GrabbyConstants.secondExtenderI, Constants.GrabbyConstants.secondExtenderD);
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
    targetPosition = secondExtender.getSelectedSensorPosition() + 2;
  }

  public void manualIn(){
    targetPosition = secondExtender.getSelectedSensorPosition() - 2;
  }

  public void stopExtending(){
    targetPosition = secondExtender.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    
    if(targetPosition > 0){
      secondExtenderSpeed = pid.calculate(secondExtender.getSelectedSensorPosition(), targetPosition);
    }else if(!photoEye.isDetecting()){
      secondExtenderSpeed = -Constants.GrabbyConstants.extenderSpeed;
    }

    failsafes();
    secondExtender.set(ControlMode.PercentOutput, secondExtenderSpeed);

  }

  private void failsafes() {
    if(secondExtender.getSelectedSensorPosition() >= Constants.GrabbyConstants.highConeState.getSecondExtendTarget() && secondExtenderSpeed > 0){
      secondExtenderSpeed = 0;
    }

    if(photoEye.isDetecting()){
      secondExtender.setSelectedSensorPosition(0);
      if(secondExtenderSpeed < 0){
        secondExtenderSpeed = 0;
      }
    }
  }
}
