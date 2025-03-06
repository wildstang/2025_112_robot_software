package org.wildstang.year2025.robot;

import org.wildstang.framework.core.Subsystems;
import org.wildstang.year2025.subsystems.Claw.Claw;
// import org.wildstang.year2025.subsystems.LED.LedController;
import org.wildstang.year2025.subsystems.LED.LedSubsystem;
import org.wildstang.year2025.subsystems.arm_lift.ArmLift;
import org.wildstang.year2025.subsystems.climb.Climb;
import org.wildstang.year2025.subsystems.localization.WsPV;
import org.wildstang.year2025.subsystems.localization.WsVision;
import org.wildstang.year2025.subsystems.swerve.SwerveDrive;

/**
 * All subsystems are enumerated here.
 * It is used in Robot.java to initialize all subsystems.
 */
public enum WsSubsystems implements Subsystems {

    // enumerate subsystems
    ARMLIFT("Arm Lift", ArmLift.class),
    WS_VISION("Ws Vision", WsVision.class),
    SWERVE_DRIVE("Swerve Drive", SwerveDrive.class),
    // LED("Led Controller", LedController.class),
    CLAW("Claw", Claw.class),
    LED("LED", LedSubsystem.class),
    CLIMB("Climb", Climb.class),
    WS_PV("Photon vision", WsPV.class)
    ;

    ;

    /**
     * Do not modify below code, provides template for enumerations.
     * We would like to have a super class for this structure, however,
     * Java does not support enums extending classes.
     */
    
    private String name;
    private Class<?> subsystemClass;

    /**
     * Initialize name and Subsystem map.
     * @param name Name, must match that in class to prevent errors.
     * @param subsystemClass Class containing Subsystem
     */
    WsSubsystems(String name, Class<?> subsystemClass) {
        this.name = name;
        this.subsystemClass = subsystemClass;
    }

    /**
     * Returns the name mapped to the subsystem.
     * @return Name mapped to the subsystem.
     */
    @Override
    public String getName() {
        return name;
    }

    /**
     * Returns subsystem's class.
     * @return Subsystem's class.
     */
    @Override
    public Class<?> getSubsystemClass() {
        return subsystemClass;
    }
}