package org.wildstang.year2025.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.year2025.robot.WsSubsystems;
import org.wildstang.year2025.subsystems.arm_lift.ArmLift;
import org.wildstang.year2025.subsystems.arm_lift.ArmLift.gameStates;

public class SetArmLiftStateStep extends AutoStep {

    private ArmLift armLift;
    private gameStates newState;

    public SetArmLiftPosStep(gameStates newState){
        this.newState = newState;
    }

    @Override
    public void initialize() {
        armLift = (ArmLift) Core.getSubsystemManager().getSubsystem(WsSubsystems.ARMLIFT);
    }

    @Override
    public void update() {
        armLift.setGameState(newState);
        setFinished();
    }

    @Override
    public String toString() {
        return "Set ArmLift State Step";
    }
    
}
