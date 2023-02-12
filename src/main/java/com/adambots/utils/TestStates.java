// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Scanner;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import com.adambots.utils.StateMachine.State;
import com.adambots.utils.StateMachine.Transition;

/** Add your docs here. */
public class TestStates {
    public final static State initState = StateMachine.getState("Init State");
    public final static State midState = StateMachine.getState("Mid State");
    public final static State topState = StateMachine.getState("Top State");

    public static class Payload{
        public double armMotorSpeed = 0;
        public int armDirection = 1; 
    }
    
    public static void main(String[] args) throws InterruptedException {
        boolean firstLimitSwitch = true;
        boolean secondLimitSwitch = true;
        double absoluteEncoderAngle = 0;
        double firstStageArmEncoderLimit = 50;
        double secondStageArmEncoderLimit = 100;
        double firstStageArmEncoderTicks = 0;
        double secondStageArmEncoderTicks = 0;
        
        initState.setTrigger(() -> (firstLimitSwitch && secondLimitSwitch && absoluteEncoderAngle == 0));
        initState.addTransition(new Transition("up", midState, (x)-> {
            ((Payload)x).armMotorSpeed = 0.5;
        }));

        midState.setTrigger(() -> (firstStageArmEncoderTicks > firstStageArmEncoderLimit));
        midState.addTransition(new Transition("up", topState, (x)->{
            armMotorSpeed = 0.5;
            armDirection = 1;
        }));

        midState.addTransition(new Transition("down", initState, (x)->{
            armMotorSpeed = 0.5;
            armDirection = -1;
        }));

        topState.setTrigger(() -> (secondStageArmEncoderTicks > secondStageArmEncoderLimit));

        ArrayList<State> states = new ArrayList<State>(){{
            add(0, initState);
            add(1, midState);
            add(2, topState);
        }};

        currentState = initState;

        // Simulate Periodic
        while (true){

            states.forEach((s) -> s.updateState());
            currentState = getCurrentState(states);

            targetState = promptForTargetState();

            Thread.sleep(1000);
        }
    }

    private static State promptForTargetState() {
        Scanner input = new Scanner(System.in);

        String str = input.nextLine();
        switch (str){
            case "up":
        }
    }

    public static State getCurrentState(ArrayList<State> states){
        for (State state : states) {
            if (state.isActive()){
                return state;
            }
        }

        return null;
    }
}
