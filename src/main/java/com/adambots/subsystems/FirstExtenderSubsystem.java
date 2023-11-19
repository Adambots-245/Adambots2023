// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants;
import com.adambots.Constants.GrabbyConstants;
import com.adambots.sensors.PhotoEye;
import com.adambots.utils.Dash;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FirstExtenderSubsystem extends SubsystemBase {

  private final TalonFX firstExtender;
  private final PIDController pid;
  private final PhotoEye photoEye;
  private final CANcoder armLifterEncoder;


  private double firstExtenderSpeed = 0;
  private double targetPosition = Constants.GrabbyConstants.initState.getFirstExtendTarget();

  public FirstExtenderSubsystem(TalonFX firstExtender, PhotoEye photoEye, CANcoder armLifterEncoder) {
    this.firstExtender = firstExtender;

    var talonFXConfigurator = firstExtender.getConfigurator();
    var motorConfigs = new MotorOutputConfigs();

    motorConfigs.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    talonFXConfigurator.apply(motorConfigs);
    
    // firstExtender.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 25, 0.1));
    this.photoEye = photoEye;
    pid = new PIDController(Constants.GrabbyConstants.firstExtenderP, Constants.GrabbyConstants.firstExtenderI, Constants.GrabbyConstants.firstExtenderD);
    this.armLifterEncoder = armLifterEncoder;
    
    Dash.add("First Extender Encoder", () -> firstExtender.getPosition().getValueAsDouble()/GrabbyConstants.armEncoderCPR);
    Dash.add("First Extender Speed", () -> firstExtenderSpeed);
    Dash.add("first PhotoEye", () -> isMaxRetracted());
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
    targetPosition = 99999999;
  }

  public void manualIn(){
    targetPosition = -99999999;
  }

  public void stopExtending(){
    targetPosition = getEncoder();
  }

  public boolean isMaxExtended () {
    return getEncoder() >= Constants.GrabbyConstants.firstExtenderMaxExtend-100;
  }

  public boolean isMaxRetracted () {
    return !photoEye.isDetecting();
  }

  public double getEncoder () {
    // firstExtender.getPosition().refresh();
    return firstExtender.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    
    if(targetPosition > 0){ //Calculate arm extension speed if target is positive
      firstExtenderSpeed = pid.calculate(getEncoder(), targetPosition);
      firstExtenderSpeed = MathUtil.clamp(firstExtenderSpeed, -Constants.GrabbyConstants.extenderSpeed, Constants.GrabbyConstants.extenderSpeed);
    }else if(!isMaxRetracted()){ //Otherwise just retract until we see photoeye or we go massively negative
      if (getEncoder() > -GrabbyConstants.firstExtenderMaxExtend) {
        firstExtenderSpeed = -GrabbyConstants.extenderSpeed;
      } else {
        firstExtenderSpeed = 0;
      }
    }
    
    failsafes();
    firstExtender.set(firstExtenderSpeed);
  }

  private void failsafes() {
    if(getEncoder() >= Constants.GrabbyConstants.firstExtenderMaxExtend-100 && firstExtenderSpeed > 0){
      firstExtenderSpeed = 0;
    }

    if(isMaxRetracted()){
      firstExtender.setPosition(0);
      if(firstExtenderSpeed < 0){
        firstExtenderSpeed = 0;
      }
    }

    if (armLifterEncoder.getAbsolutePosition().getValueAsDouble() <= Constants.GrabbyConstants.groundLifterValue + 10) {
      targetPosition = 0;
    } 
  }
}
