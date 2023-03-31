// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants.GrabbyConstants;
import com.adambots.sensors.Lidar;
import com.ctre.phoenix.sensors.CANCoder;
import com.adambots.actuators.StepperMotor;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabSubsystem extends SubsystemBase {
  
  private DoubleSolenoid grabby;
  private StepperMotor grabbyStepper;
  private WPI_CANCoder lifterEncoder;
  private Lidar lidar;
  private double prevDist = 999;

  // private double currentLifterVal;

  // public GrabSubsystem(DoubleSolenoid grabby, StepperMotor grabbyStepper, WPI_CANCoder lifterEncoder, Lidar lidar) {
  public GrabSubsystem(DoubleSolenoid grabby, WPI_CANCoder lifterEncoder, Lidar lidar) {
    this.grabby = grabby;
    this.grabbyStepper = grabbyStepper;
    this.lifterEncoder = lifterEncoder;
    this.lidar = lidar;
  }

  public void grab(){
    grabby.set(Value.kForward);
  }

  public void ungrab(){
    grabby.set(Value.kReverse);
  }

  public void stepUp(){
    grabbyStepper.stepUp(20);
  }
  
  public void stepDown(){
    grabbyStepper.stepDown(20);
  }

  @Override
  public void periodic() {
    // if (lifterEncoder.getAbsolutePosition() <= (GrabbyConstants.groundLifterValue + 10) && lidar.getInches() < 7 && prevDist >= 7 && lidar.getInches() > 2.5) {
    //   grab();
    // }
    prevDist = lifterEncoder.getAbsolutePosition();

    // if(Math.abs(currentLifterVal - lifterEncoder.getAbsolutePosition()) > 3){
    //   grabbyStepper.stepUp(currentLifterVal - lifterEncoder.getAbsolutePosition());
    //   currentLifterVal = lifterEncoder.getAbsolutePosition();
    // }
  }
}
