// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants;
import com.adambots.Constants.GrabbyConstants;
import com.adambots.sensors.PhotoEye;
import com.adambots.utils.Dash;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SecondExtenderSubsystem extends SubsystemBase {
  
  private final TalonFX secondExtender;
  private final PIDController pid;
  private final PhotoEye photoEye;
  private final WPI_CANCoder armLifterEncoder;


  private double secondExtenderSpeed = 0;
  private double targetPosition = GrabbyConstants.initState.getSecondExtendTarget();
  private double maxSpeed = GrabbyConstants.extenderSpeed;
  private double offset = 0;

  private double maxTotal;

  public SecondExtenderSubsystem(TalonFX secondExtender, PhotoEye photoEye, WPI_CANCoder armLifterEncoder) {
    this.secondExtender = secondExtender;
    secondExtender.setInverted(true);
    secondExtender.setNeutralMode(NeutralMode.Brake);

    this.photoEye = photoEye;
    pid = new PIDController(Constants.GrabbyConstants.secondExtenderP, Constants.GrabbyConstants.secondExtenderI, Constants.GrabbyConstants.secondExtenderD);
    this.armLifterEncoder = armLifterEncoder;

    Dash.add("Second Extender Encoder", () -> secondExtender.getSelectedSensorPosition()/GrabbyConstants.armEncoderCPR);
    Dash.add("Second Extender Speed", () -> secondExtenderSpeed);
    Dash.add("Second PhotoEye", () -> isMaxRetracted());
  }

  public void changeMaxSpeed(double newMax){
    maxSpeed = newMax;
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

  public void setOffset (double offset) {
    this.offset = offset;
  }

  public void addOffset (double offset) {
    this.offset += offset;
    offset = MathUtil.clamp(offset, -5, 5);
  }

  @Override
  public void periodic() {
    maxTotal = (28.646 + 152.727)/Math.cos(Math.toRadians(armLifterEncoder.getAbsolutePosition() + GrabbyConstants.limitOffset + offset)); //limits horizontal max extension
    if(maxTotal <= 62 + (targetPosition/GrabbyConstants.armEncoderCPR) + 89.09){
      targetPosition = (maxTotal - (62 + 89.09 - 6))*GrabbyConstants.armEncoderCPR;
    }

    if(targetPosition > 0){ //Calculate arm extension speed if target is positive
      secondExtenderSpeed = pid.calculate(secondExtender.getSelectedSensorPosition(), targetPosition);
      secondExtenderSpeed = MathUtil.clamp(secondExtenderSpeed, -maxSpeed, maxSpeed);
    }else if(!isMaxRetracted()){ //Otherwise just retract until we see photoeye or we go massively negative
      if (secondExtender.getSelectedSensorPosition() > -GrabbyConstants.secondExtenderMaxExtend) {
        secondExtenderSpeed = -maxSpeed;
      } else {
        secondExtenderSpeed = 0;
      }
    }

    failsafes();
    secondExtender.set(ControlMode.PercentOutput, secondExtenderSpeed);
    // secondExtender.set(ControlMode.PercentOutput, 0);
  }

  private void failsafes() {
    //Preventing the arm from going too far out or in

    // if(secondExtender.getSelectedSensorPosition() > GrabbyConstants.horizontalMaxEncoderValue && armLifterEncoder.getAbsolutePosition()+GrabbyConstants.limitOffset < 5){
    //   secondExtenderSpeed = -GrabbyConstants.extenderSpeed;
    // }

    if(secondExtender.getSelectedSensorPosition() > GrabbyConstants.veritcalMaxEncoderValue-GrabbyConstants.armEncoderCPR*3 && armLifterEncoder.getAbsolutePosition() > 200){ //limits vertical max entension
      if (secondExtenderSpeed > 0) {
        secondExtenderSpeed = 0;
      }
    }
    if(secondExtender.getSelectedSensorPosition() > GrabbyConstants.veritcalMaxEncoderValue && armLifterEncoder.getAbsolutePosition() > 200){ //also limits vertical max entension
      secondExtenderSpeed = -GrabbyConstants.extenderSpeed;
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
