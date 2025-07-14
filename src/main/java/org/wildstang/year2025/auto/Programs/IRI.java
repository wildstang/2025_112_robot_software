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
public class IRI extends AutoProgram {

    @Override
    protected void defineSteps() {
        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);

        AutoParallelStepGroup group1 = new AutoParallelStepGroup();
        group1.addStep(new SwervePathFollowerStep("IRI_1", swerve, true));
        AutoSerialStepGroup group1_1 = new AutoSerialStepGroup();
        AutoParallelStepGroup group1_1_1 = new AutoParallelStepGroup();
        group1_1_1.addStep(new SetArmLiftStateStep(GameStates.GROUND_INTAKE, true));
        group1_1_1.addStep(new SetClawStateStep(clawStates.INTAKE));
        group1_1.addStep(group1_1_1);
        group1_1.addStep(new SetArmLiftStateStep(GameStates.SHOOT_NET, false));
        group1.addStep(group1_1);
        addStep(group1);

        addStep(new AutoStepDelay(200));
        addStep(new SetClawStateStep(clawStates.OUTTAKE));

        AutoParallelStepGroup group2 = new AutoParallelStepGroup();
        group2.addStep(new SwervePathFollowerStep("IRI_2", swerve, true));
        AutoSerialStepGroup group2_1 = new AutoSerialStepGroup();
        AutoParallelStepGroup group2_1_1 = new AutoParallelStepGroup();
        group2_1_1.addStep(new SetArmLiftStateStep(GameStates.GROUND_INTAKE, true));
        group2_1_1.addStep(new SetClawStateStep(clawStates.INTAKE));
        group2_1.addStep(group2_1_1);
        group2_1.addStep(new SetArmLiftStateStep(GameStates.SHOOT_NET, false));
        group2.addStep(group2_1);
        addStep(group2);

        addStep(new AutoStepDelay(200));
        addStep(new SetClawStateStep(clawStates.OUTTAKE));

        AutoParallelStepGroup group3 = new AutoParallelStepGroup();
        group3.addStep(new SwervePathFollowerStep("IRI_3", swerve, true));
        AutoSerialStepGroup group3_1 = new AutoSerialStepGroup();
        AutoParallelStepGroup group3_1_1 = new AutoParallelStepGroup();
        group3_1_1.addStep(new SetArmLiftStateStep(GameStates.L3_ALGAE_REEF, true));
        group3_1_1.addStep(new SetClawStateStep(clawStates.INTAKE));
        group3_1.addStep(group3_1_1);
        group3_1.addStep(new SetArmLiftStateStep(GameStates.SHOOT_NET, false));
        group3.addStep(group3_1);
        addStep(group3);

        addStep(new AutoStepDelay(200));
        addStep(new SetClawStateStep(clawStates.OUTTAKE));

        AutoParallelStepGroup group4 = new AutoParallelStepGroup();
        group4.addStep(new SwervePathFollowerStep("IRI_4", swerve, true));
        AutoSerialStepGroup group4_1 = new AutoSerialStepGroup();
        AutoParallelStepGroup group4_1_1 = new AutoParallelStepGroup();
        group4_1_1.addStep(new SetArmLiftStateStep(GameStates.L2_ALGAE_REEF, true));
        group4_1_1.addStep(new SetClawStateStep(clawStates.INTAKE));
        group4_1.addStep(group4_1_1);
        group4_1.addStep(new SetArmLiftStateStep(GameStates.SHOOT_NET, false));
        group4.addStep(group4_1);
        addStep(group4);

        addStep(new AutoStepDelay(200));
        addStep(new SetClawStateStep(clawStates.OUTTAKE));

        addStep(new SwervePathFollowerStep("IRI_5", swerve, true));
    }

    @Override
    public String toString() {
        return "IRI";
    }
    
}