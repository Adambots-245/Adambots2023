// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants.GrabbyConstants;
import com.adambots.sensors.Lidar;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabSubsystem extends SubsystemBase {
  DoubleSolenoid grabby;
  CANCoder lifterEncoder;
  Lidar lidar;
  double prevDist = 999;

  public GrabSubsystem(DoubleSolenoid grabby, CANCoder lifterEncoder, Lidar lidar) {
    this.grabby = grabby;
    this.lifterEncoder = lifterEncoder;
    this.lidar = lidar;
  }

  public void grab(){
    grabby.set(Value.kForward);
  }

  public void ungrab(){
    grabby.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    if (lifterEncoder.getAbsolutePosition() <= (GrabbyConstants.groundLifterValue + 10) && lidar.getInches() < 7 && prevDist >= 7 && lidar.getInches() > 2.5) {
      grab();
    }
    prevDist = lifterEncoder.getAbsolutePosition();
  }
}
