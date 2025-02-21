package org.wildstang.year2025.subsystems.localization;

// ton of imports
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.year2025.robot.WsSubsystems;
import org.wildstang.year2025.subsystems.swerve.SwerveDrive;
import org.wildstang.framework.core.Core;

import org.wildstang.framework.io.inputs.Input;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WsVision implements Subsystem {
    
    public SwerveDrive swerve;

    private Timer lastUpdate = new Timer();

    @Override
    public void init() {
        lastUpdate.start();
    }

    @Override
    public void initSubsystems() {
        swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
    }

    @Override
    public void inputUpdate(Input source) {
    }

    @Override
    public void selfTest() {
    }

    @Override
    public void update() {
        if (aprilTagsInView()) lastUpdate.reset();
        SmartDashboard.putBoolean("Vision targetinView", aprilTagsInView());
    }

    /**
     * can either camera see an April Tag
     */ 
    public boolean aprilTagsInView(){
        // return left.TargetInView() || right.TargetInView();
        return false;
    }

    public double getUpdateTime(){
        return lastUpdate.get();
    }

    public Translation2d getCameraPose(){
        // if (isLeftBetter()) return new Translation2d(left.target3D[0], left.target3D[1]);
        // else return new Translation2d(right.target3D[0], right.target3D[1]);
        return new Translation2d();
    }

    @Override
    public void resetState() {
    }

    @Override
    public String getName() {
        return "Ws Vision";
    }
}