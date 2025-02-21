package org.wildstang.year2025.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.year2025.robot.WsSubsystems;
import org.wildstang.year2025.subsystems.Claw.Claw;
import org.wildstang.year2025.subsystems.Claw.Claw.clawStates;

public class SetClawStateStep extends AutoStep {

    private Claw claw;
    private clawStates newState;

    public SetArmLiftPosStep(clawStates newState){
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
        return "Set Claw State Step";
    }
    
}
