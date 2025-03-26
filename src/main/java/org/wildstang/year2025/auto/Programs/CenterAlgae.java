package org.wildstang.year2025.auto.Programs;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.auto.steps.AutoSerialStepGroup;
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
public class CenterAlgae extends AutoProgram {

    @Override
    protected void defineSteps() {
        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);

        AutoParallelStepGroup group1 = new AutoParallelStepGroup();
        group1.addStep(new SetArmLiftStateStep(GameStates.L2_ALGAE_REEF, true));
        group1.addStep(new SwervePathFollowerStep("CenterStartToGH", swerve, true));
        group1.addStep(new SetClawStateStep(clawStates.INTAKE));
        addStep(group1);

        addStep(new AutoStepDelay(250));

        AutoParallelStepGroup group2 = new AutoParallelStepGroup();
        group2.addStep(new SwervePathFollowerStep("GHToBarge", swerve));
        group2.addStep(new SetArmLiftStateStep(GameStates.SHOOT_NET, false));
        addStep(group2);

        addStep(new SetClawStateStep(clawStates.OUTTAKE));

        AutoParallelStepGroup group3 = new AutoParallelStepGroup();
        group3.addStep(new SetArmLiftStateStep(GameStates.L3_ALGAE_REEF, true));
        group3.addStep(new SwervePathFollowerStep("BargeToIJ", swerve));
        group3.addStep(new SetClawStateStep(clawStates.INTAKE));
        addStep(group3);

        AutoParallelStepGroup group4 = new AutoParallelStepGroup();
        group4.addStep(new SwervePathFollowerStep("IJToBarge", swerve));
        group4.addStep(new SetArmLiftStateStep(GameStates.SHOOT_NET, false));
        addStep(group4);

        addStep(new SetClawStateStep(clawStates.OUTTAKE));

        AutoParallelStepGroup group5 = new AutoParallelStepGroup();
        group5.addStep(new SetArmLiftStateStep(GameStates.L2_ALGAE_REEF, true));
        group5.addStep(new SwervePathFollowerStep("BargeToKL", swerve));
        group5.addStep(new SetClawStateStep(clawStates.INTAKE));
        addStep(group5);

        AutoParallelStepGroup group6 = new AutoParallelStepGroup();
        group6.addStep(new SwervePathFollowerStep("KLToBarge", swerve));
        group6.addStep(new SetArmLiftStateStep(GameStates.SHOOT_NET, false));
        addStep(group6);

        addStep(new AutoStepDelay(500));
        addStep(new SetClawStateStep(clawStates.OUTTAKE));

    }

    @Override
    public String toString() {
        return "Center Algae";
    }
    
}