// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.actuators.StepperMotor;
import com.adambots.actuators.StepperMotorPWM;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabSubsystem extends SubsystemBase {
    private DoubleSolenoid grabby;
    private StepperMotorPWM grabbyStepper;

  public GrabSubsystem(DoubleSolenoid grabby, StepperMotorPWM grabbyStepper) {
    this.grabby = grabby;
    this.grabbyStepper = grabbyStepper;
  }

  public void grab(){
    grabby.set(Value.kForward);
  }

  public void ungrab(){
    grabby.set(Value.kReverse);
  }

  public void stepUp(){
    grabbyStepper.setSpeed(0.50);
  }
  
  public void stepDown(){
    grabbyStepper.setSpeed(-0.50);
  }

  public void stop(){
    grabbyStepper.stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
