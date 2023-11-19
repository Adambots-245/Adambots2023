/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

public class ChangeStream extends Command {
  
  public ChangeStream() {
  //  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
     System.out.println("hi");

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
