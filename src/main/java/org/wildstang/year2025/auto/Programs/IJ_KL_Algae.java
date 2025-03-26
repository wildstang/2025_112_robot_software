package org.wildstang.year2025.auto.Programs;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.auto.steps.SwervePathFollowerStep;
import org.wildstang.framework.auto.steps.control.AutoStepDelay;
import org.wildstang.framework.core.Core;
import org.wildstang.year2025.auto.Steps.SetArmLiftStateStep;
import org.wildstang.year2025.auto.Steps.SetClawStateStep;
import org.wildstang.year2025.robot.WsSubsystems;
import org.wildstang.year2025.subsystems.Claw.Claw.clawStates;
import org.wildstang.year2025.subsystems.arm_lift.ArmLift.GameStates;
import org.wildstang.year2025.subsystems.swerve.SwerveDrive;

/**
 */
public class IJ_KL_Algae extends AutoProgram {

    @Override
    protected void defineSteps() {
        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);

        AutoParallelStepGroup group1 = new AutoParallelStepGroup();
        group1.addStep(new SetArmLiftStateStep(GameStates.L3_ALGAE_REEF, true));
        group1.addStep(new SwervePathFollowerStep("IJStartToIJ", swerve, true));
        group1.addStep(new SetClawStateStep(clawStates.INTAKE));
        addStep(group1);

        AutoParallelStepGroup group2 = new AutoParallelStepGroup();
        group2.addStep(new SwervePathFollowerStep("IJToBarge", swerve));
        group2.addStep(new SetArmLiftStateStep(GameStates.SHOOT_NET, false));
        addStep(group2);

        addStep(new AutoStepDelay(200));
        addStep(new SetClawStateStep(clawStates.OUTTAKE));
        addStep(new SwervePathFollowerStep("BargeToKL", swerve));
    }

    @Override
    public String toString() {
        return "IJ KL Algae";
    }
    
}