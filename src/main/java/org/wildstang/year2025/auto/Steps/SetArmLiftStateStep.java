package org.wildstang.year2025.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.year2025.robot.WsSubsystems;
import org.wildstang.year2025.subsystems.arm_lift.ArmLift;
import org.wildstang.year2025.subsystems.arm_lift.ArmLift.GameStates;

public class SetArmLiftStateStep extends AutoStep {

    private ArmLift armLift;
    private GameStates newState;
    private boolean isFront;

    public SetArmLiftStateStep(GameStates newState, boolean isFront){
        this.newState = newState;
        this.isFront = isFront;
    }

    @Override
    public void initialize() {
        armLift = (ArmLift) Core.getSubsystemManager().getSubsystem(WsSubsystems.ARMLIFT);
        armLift.setGameState(newState, isFront);
    }

    @Override
    public void update() {
        if (armLift.isAtSetpoint()){
            setFinished();
        }
    }

    @Override
    public String toString() {
        return "Set ArmLift State Step";
    }
    
}
