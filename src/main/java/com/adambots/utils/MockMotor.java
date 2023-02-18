// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

/** Add your docs here. */
public class MockMotor {

    private double motorSpeed = 0.0;
    private MockCancoder mockEncoder = null;
    private double integratedEncoderTicks = 0.0;

    public MockMotor(){

    }

    public MockMotor(MockCancoder mockEncoder) {
        this.mockEncoder = mockEncoder;
    }

    public void setInverted(boolean b) {
    }

    public void setNeutralMode(NeutralMode brake) {
    }

    public double getSelectedSensorPosition(){
        return integratedEncoderTicks;
    }

    public void set(ControlMode percentoutput, double speed) {
        if (speed != 0) {
            
            if (mockEncoder != null) {
                if (speed > 0) {
                    mockEncoder.decrementAngle(5);
                } else {
                    
                    mockEncoder.incrementAngle(5);
                }
            }
            
            if (speed > 0){
                integratedEncoderTicks += 0.2;
            } else {
                integratedEncoderTicks -= 0.2;
            }

            System.out.println("Motor running: " + speed + " (" + integratedEncoderTicks + ")");
        } else if (motorSpeed != 0.0 && speed == 0.0) {
            System.out.println("Stopping motor!");
        }

        motorSpeed = speed;
    }

    public void setSelectedSensorPosition(int i) {
        integratedEncoderTicks = 0.0;
    }

}
