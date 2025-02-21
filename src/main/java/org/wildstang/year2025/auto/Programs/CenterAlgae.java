package org.wildstang.year2025.auto.Programs;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.SwervePathFollowerStep;
import org.wildstang.framework.core.Core;
import org.wildstang.year2025.robot.WsSubsystems;
import org.wildstang.year2025.subsystems.swerve.SwerveDrive;

/**
 */
public class CenterAlgae extends AutoProgram {

    @Override
    protected void defineSteps() {
        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        addStep(new SwervePathFollowerStep("CenterStartToGH", swerve, true));
        addStep(new SwervePathFollowerStep("GHToBarge", swerve));
    }

    @Override
    public String toString() {
        return "Center Algae";
    }
    
}