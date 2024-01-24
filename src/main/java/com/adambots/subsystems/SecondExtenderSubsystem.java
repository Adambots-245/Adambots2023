// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants;
import com.adambots.Constants.GrabbyConstants;
import com.adambots.utils.Dash;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SecondExtenderSubsystem extends SubsystemBase {
  
  private final TalonFX secondExtender;
  private final PIDController pid;
  private final DigitalInput photoEye;
  private final CANcoder armLifterEncoder;


  private double secondExtenderSpeed = 0;
  private double targetPosition = GrabbyConstants.initState.getSecondExtendTarget();
  private double maxSpeed = GrabbyConstants.extenderSpeed;
  private double offset = 0;

  private double maxTotal;

  public SecondExtenderSubsystem(TalonFX secondExtender, DigitalInput photoEye, CANcoder armLifterEncoder) {
    this.secondExtender = secondExtender;

    var talonFXConfigurator = secondExtender.getConfigurator();
    var motorConfigs = new MotorOutputConfigs();

    motorConfigs.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    talonFXConfigurator.apply(motorConfigs);

    this.photoEye = photoEye;
    pid = new PIDController(Constants.GrabbyConstants.secondExtenderP, Constants.GrabbyConstants.secondExtenderI, Constants.GrabbyConstants.secondExtenderD);
    this.armLifterEncoder = armLifterEncoder;

    Dash.add("Second Extender Encoder", () -> secondExtender.getPosition().getValueAsDouble()/GrabbyConstants.armEncoderCPR);
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
    targetPosition = getEncoder();
  }

  public boolean isMaxRetracted () {
    return !photoEye.get();
  }

  public void setOffset (double offset) {
    this.offset = offset;
  }

  public void addOffset (double offset) {
    this.offset += offset;
    offset = MathUtil.clamp(offset, -5, 5);
  }

  public double getEncoder () {
    // secondExtender.getPosition().refresh();
    return secondExtender.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    maxTotal = (28.646 + 152.727)/Math.cos(Math.toRadians(getEncoder() + GrabbyConstants.limitOffset + offset)); //limits horizontal max extension
    if(maxTotal <= 62 + (targetPosition/GrabbyConstants.armEncoderCPR) + 89.09){
      targetPosition = (maxTotal - (62 + 89.09 - 6))*GrabbyConstants.armEncoderCPR;
    }

    if(targetPosition > 0){ //Calculate arm extension speed if target is positive
      secondExtenderSpeed = pid.calculate(getEncoder(), targetPosition);
      secondExtenderSpeed = MathUtil.clamp(secondExtenderSpeed, -maxSpeed, maxSpeed);
    }else if(!isMaxRetracted()){ //Otherwise just retract until we see photoeye or we go massively negative (stops constant belt slipping from maxRetraction sensor failure)
      if (getEncoder() > -GrabbyConstants.secondExtenderMaxExtend) {
        secondExtenderSpeed = -maxSpeed;
      } else {
        secondExtenderSpeed = 0;
      }
    }

    failsafes();
    // secondExtender.set(secondExtenderSpeed);
    secondExtender.set(0);
  }

  private void failsafes() {
    //limits max vertical entension
    if(getEncoder() > GrabbyConstants.veritcalMaxEncoderValue-GrabbyConstants.armEncoderCPR*3 && armLifterEncoder.getAbsolutePosition().getValueAsDouble() > 200){
      if (secondExtenderSpeed > 0) {
        secondExtenderSpeed = 0;
      }
    }

    //drives arm back in if it exceeds max vertical extension
    if(getEncoder() > GrabbyConstants.veritcalMaxEncoderValue && armLifterEncoder.getAbsolutePosition().getValueAsDouble() > 200){ 
      secondExtenderSpeed = -GrabbyConstants.extenderSpeed;
    }

    //limits max extenion
    if(getEncoder() >= Constants.GrabbyConstants.secondExtenderMaxExtend && secondExtenderSpeed > 0){
      secondExtenderSpeed = 0;
    }

    //Reset secondExtender encoder when max retraction is detected - May have negative impact if max retraction switch fails (use rising edge detection to mitigate)
    if(isMaxRetracted()){
      secondExtender.setPosition(0);
      if(secondExtenderSpeed < 0){
        secondExtenderSpeed = 0;
      }
    }

    //Preventing the arm from extending into the ground
    if (armLifterEncoder.getAbsolutePosition().getValueAsDouble() <= Constants.GrabbyConstants.groundLifterValue + 10) {
        targetPosition = 0;
    } 
  }
}
