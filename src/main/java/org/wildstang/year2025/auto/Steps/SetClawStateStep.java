package org.wildstang.year2025.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.year2025.robot.WsSubsystems;
import org.wildstang.year2025.subsystems.Claw.Claw;
import org.wildstang.year2025.subsystems.Claw.Claw.clawStates;
import org.wildstang.year2025.subsystems.arm_lift.ArmLift;

public class SetClawStateStep extends AutoStep {

    private Claw claw;
    private clawStates newState;

    public SetClawStateStep(clawStates newState){
        this.newState = newState;
    }

    @Override
    public void initialize() {
        claw = (Claw) Core.getSubsystemManager().getSubsystem(WsSubsystems.CLAW);
    }

    @Override
    public void update() {
        claw.setGameState(newState);
        setFinished();
    }

    @Override
    public String toString() {
        return "Set Claw State Step";
    }
    
}
