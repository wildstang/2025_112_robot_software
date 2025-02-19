package org.wildstang.year2025.auto.Programs;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.SwervePathFollowerStep;
import org.wildstang.framework.core.Core;
import org.wildstang.year2025.auto.Steps.AutoSetupStep;
import org.wildstang.year2025.robot.WsSubsystems;
import org.wildstang.year2025.subsystems.swerve.SwerveDrive;


import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class TestProgram extends AutoProgram{
    
    protected void defineSteps(){
        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        addStep(new AutoSetupStep(0.7, 4.0, 180.0, Alliance.Blue));
        addStep(new SwervePathFollowerStep("TestSlow", swerve));
        
 
    }

    public String toString(){
        return "Test Program";
    }
}