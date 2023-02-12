// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.utils;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

/** Add your docs here. */
public class StateMachine{
    public Object Payload;
    private Map<String, State> states = new HashMap<>();
    private static StateMachine _instance = new StateMachine();

    public static class State{
        private StateMachine _machine;
        
        private String name;
        private boolean active = false;
        private BooleanSupplier trigger;
        private Map<String, Transition> transitions;
        
        public Transition getTransition(String name){
            return transitions.getOrDefault(name, null);
        }
        
        public void addTransition(Transition transition) {
            transitions.put(transition.name, transition);
        }
        
        private State(String name) {
            this.name = name;
        }
        
        private State(String name, boolean active) {
            this.name = name;
            this.active = active;
        }
        
        public StateMachine get_machine() {
            return _machine;
        }

        public void setTrigger(BooleanSupplier trigger) {
            this.trigger = trigger;
        }

        public String getName() {
            return name;
        }

        public boolean isActive() {
            updateState();
            return active;
        }

        public void updateState(){
            active = trigger.getAsBoolean();
            System.out.printf("Updated State for %s: %b\n", name, active);
        }
    }

    public static class Transition<T>{
        private String name;
        private State target;
        private Consumer<T> action;
        private Object payload;

        public Consumer<T> getAction() {
            return action;
        }
        public void setAction(Consumer<T> action) {
            this.action = action;
        }
        public Transition(String name, State target, Consumer<T> action) {
            this.name = name;
            this.target = target;
            this.action = action;
        }
        public String getName() {
            return name;
        }
        public void setName(String name) {
            this.name = name;
        }
        public State getTarget() {
            return target;
        }
        public void setTarget(State target) {
            this.target = target;
        }

        public void act(){
            this.action.accept((T)target.get_machine<T>().Payload);
        }
    }

    public static State getState(String name){
        State state = new State(name);
        state._machine = InstanceHolder.instance;
        state._machine.states.put(name, state);

        return state;
    }
}
