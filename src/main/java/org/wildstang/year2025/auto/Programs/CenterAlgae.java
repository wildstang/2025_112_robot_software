package org.wildstang.year2025.auto.Programs;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelFinishedOnAnyStepGroup;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.auto.steps.SwervePathFollowerStep;
import org.wildstang.framework.core.Core;
import org.wildstang.year2025.auto.Steps.SetArmLiftStateStep;
import org.wildstang.year2025.auto.Steps.SetClawStateStep;
import org.wildstang.year2025.robot.WsSubsystems;
import org.wildstang.year2025.subsystems.Claw.Claw;
import org.wildstang.year2025.subsystems.Claw.Claw.clawStates;
import org.wildstang.year2025.subsystems.arm_lift.ArmLift;
import org.wildstang.year2025.subsystems.arm_lift.ArmLift.gameStates;
import org.wildstang.year2025.subsystems.swerve.SwerveDrive;

/**
 */
public class CenterAlgae extends AutoProgram {

    @Override
    protected void defineSteps() {
        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        ArmLift armLift = (ArmLift) Core.getSubsystemManager().getSubsystem((WsSubsystems.ARMLIFT));
        Claw claw = (Claw) Core.getSubsystemManager().getSubsystem(WsSubsystems.CLAW);
        addStep(new SwervePathFollowerStep("CenterStartToGH", swerve, true));


        AutoParallelStepGroup group1 = new AutoParallelStepGroup();
        group1.addStep(new SetArmLiftStateStep(gameStates.GROUND_INTAKE));
        group1.addStep(new SetClawStateStep(clawStates.INTAKE));
        addStep(new SwervePathFollowerStep("GHToBarge", swerve));

        AutoParallelStepGroup group2 = new AutoParallelStepGroup();
        group2.addStep(new SetArmLiftStateStep(gameStates.SHOOT_NET));
        group2.addStep(new SetClawStateStep(clawStates.OUTTAKE));
    }

    @Override
    public String toString() {
        return "Center Algae";
    }
    
}