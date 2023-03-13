// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import com.adambots.sensors.Gyro;
import com.adambots.sensors.UltrasonicSensor;
import com.adambots.subsystems.DrivetrainSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToDistanceCommand extends CommandBase {
  private DrivetrainSubsystem drivetrainSubsystem;
  private UltrasonicSensor ultrasonic;
  private Gyro gyro;

  private int minDist = 37;
  private int maxDist = 75;

  /** Creates a new DriveToDistanceCommand. */
  public DriveToDistanceCommand(DrivetrainSubsystem drivetrainSubsystem, UltrasonicSensor ultrasonic, Gyro gyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.ultrasonic = ultrasonic;
    this.gyro = gyro;

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = 180/360;
    double y = gyro.getYaw()/360;
    double rot = ((x-y) - Math.floor(x-y + 0.5)) * 0.15;

    if (ultrasonic.getInches() > 35.5+3) {
      double speed = (ultrasonic.getInches()-minDist)/(maxDist-minDist);
      drivetrainSubsystem.drive(MathUtil.interpolate(0.1, 0.4, speed), 0, rot, true);
    } else if (ultrasonic.getInches() < 27) {
      drivetrainSubsystem.drive(-0.1, 0, 0, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
    if (interrupted) {
      System.out.println("DriveToDistanceCommand Interrupted");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ultrasonic.getInches() <= 35.5 && ultrasonic.getInches() >= 27;
  }
}
