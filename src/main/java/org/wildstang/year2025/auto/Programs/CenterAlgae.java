package org.wildstang.year2025.auto.Programs;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.auto.steps.SwervePathFollowerStep;
import org.wildstang.framework.core.Core;
import org.wildstang.year2025.auto.Steps.SetArmLiftStateStep;
import org.wildstang.year2025.auto.Steps.SetClawStateStep;
import org.wildstang.year2025.robot.WsSubsystems;
import org.wildstang.year2025.subsystems.Claw.Claw.clawStates;
import org.wildstang.year2025.subsystems.arm_lift.ArmLift.gameStates;
import org.wildstang.year2025.subsystems.swerve.SwerveDrive;

/**
 */
public class CenterAlgae extends AutoProgram {

    @Override
    protected void defineSteps() {
        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);

        addStep(new SetArmLiftStateStep(gameStates.L2_ALGAE_REEF, true));
        AutoParallelStepGroup group1 = new AutoParallelStepGroup();
        group1.addStep(new SwervePathFollowerStep("CenterStartToGH", swerve, true));
        group1.addStep(new SetClawStateStep(clawStates.INTAKE));
        addStep(group1);

        AutoParallelStepGroup group2 = new AutoParallelStepGroup();
        group2.addStep(new SwervePathFollowerStep("GHToBarge", swerve));
        group2.addStep(new SetArmLiftStateStep(gameStates.SHOOT_NET, false));
        addStep(group2);

        addStep(new SetClawStateStep(clawStates.OUTTAKE));
    }

    @Override
    public String toString() {
        return "Center Algae";
    }
    
}