// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/** Add your docs here. */
public class StructureStates {

    public enum structureState {
        stored, intake, amp, subwoof, podium, commonSpeaker, startup
    }

    public static structureState currentState;

    // sets curentstate to stored
    public static void setCurrentState(structureState newstate) {
        currentState = newstate;
    }

    // returns currentstate
    public static structureState getCurrentState() {
        return currentState;
    }
}
