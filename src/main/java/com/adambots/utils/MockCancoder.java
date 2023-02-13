// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.utils;

/** Add your docs here. */
public class MockCancoder {

    private double angle = 0.0;

    public MockCancoder(double initialAngle) {
        this.angle = initialAngle;
    }

    public void setAngle(double val){
        angle = val;
    }

    public void incrementAngle(double val){
        angle += val;
    }

    public void decrementAngle(double val){
        angle -= val;
    }

    public double getAbsolutePosition() {
        // System.out.println("Absolute Encoder Value: " + angle);
        return angle;
    }

}
