// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabSubsystem extends SubsystemBase {
    DoubleSolenoid grabby;
  public GrabSubsystem(DoubleSolenoid grabby) {
    this.grabby = grabby;
  }

  public void grab(){
    grabby.set(Value.kForward);
  }

  public void ungrab(){
    grabby.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
