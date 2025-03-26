package org.wildstang.year2025.robot;

import org.wildstang.framework.core.AutoPrograms;
import org.wildstang.year2025.auto.Programs.CenterAlgae;
import org.wildstang.year2025.auto.Programs.EF_GH_Algae;
import org.wildstang.year2025.auto.Programs.IJ_KL_Algae;


/**
 * All active AutoPrograms are enumerated here.
 * It is used in Robot.java to initialize all programs.
 */
public enum WsAutoPrograms implements AutoPrograms {

    // enumerate programs
    //SAMPLE_PROGRAM("Sample", SampleAutoProgram.class),
    CENTER_ALGAE("Center Algae", CenterAlgae.class),
    IJ_KL_ALGAE("IJ KL Algae", IJ_KL_Algae.class),
    EF_GH_ALGAE("EF GH Algae", EF_GH_Algae.class),
    ;

    /**
     * Do not modify below code, provides template for enumerations.
     * We would like to have a super class for this structure, however,
     * Java does not support enums extending classes.
     */
    
    private String name;
    private Class<?> programClass;

    /**
     * Initialize name and AutoProgram map.
     * @param name Name, must match that in class to prevent errors.
     * @param programClass Class containing AutoProgram
     */
    WsAutoPrograms(String name, Class<?> programClass) {
        this.name = name;
        this.programClass = programClass;
    }

    /**
     * Returns the name mapped to the AutoProgram.
     * @return Name mapped to the AutoProgram.
     */
    @Override
    public String getName() {
        return name;
    }

    /**
     * Returns AutoProgram's class.
     * @return AutoProgram's class.
     */
    @Override
    public Class<?> getProgramClass() {
        return programClass;
    }
}