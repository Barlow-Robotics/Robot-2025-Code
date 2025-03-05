// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmState;

/** Add your docs here. */
public class ArmStateManager {

    //wpk maybe make this whole thing static?

    private ArmState currentState = ArmState.Running ;
    private ArmState targetState = currentState  ;


    public ArmStateManager() {
    }


    public ArmState getCurrentState() {
        return currentState ;
    }

    public void setCurrentState( ArmState s) {
        currentState = s ;
    }


    public ArmState getTargetState() {
        return targetState ;
    }

    public void setTargetState( ArmState s) {
        targetState = s ;
    }


    public boolean isInTransition() {
        return currentState != targetState ;
    }


    public boolean isAvailableToGoToReef() {
        return currentState != ArmState.WaitingForCoral ;
    }

    public boolean isAvailableToGoToCoralStation() {
        return currentState == ArmState.WaitingForCoral ;
    }








}
