package org.wildstang.year2025.robot;

// expand this and edit if trouble with Ws
import org.wildstang.framework.core.Core;
import org.wildstang.framework.core.Outputs;
import org.wildstang.framework.hardware.OutputConfig;
import org.wildstang.framework.io.outputs.Output;
import org.wildstang.hardware.roborio.outputs.config.WsMotorControllers;
import org.wildstang.hardware.roborio.outputs.config.WsSparkConfig;
import org.wildstang.hardware.roborio.outputs.config.WsSparkFollowerConfig;

/**
 * Output mappings are stored here.
 * Below each Motor, PWM, Digital Output, Solenoid, and Relay is enumerated with their appropriated IDs.
 * The enumeration includes a name, output type, and output config object.
 */
public enum WsOutputs implements Outputs {

    // ---------------------------------
    // Drive Motors
    // ---------------------------------
    
    // ---------------------------------
    // Other Motors
    // ---------------------------------

    // ---------------------------------
    // Solenoids
    // ---------------------------------

    // ---------------------------------
    // Other
    // ---------------------------------

    ; // end of enum

    /**
     * Do not modify below code, provides template for enumerations.
     * We would like to have a super class for this structure, however,
     * Java does not support enums extending classes.
     */

    private String m_name;
    private OutputConfig m_config;

    /**
     * Initialize a new Output.
     * @param p_name Name, must match that in class to prevent errors.
     * @param p_config Corresponding configuration for OutputType.
     */
    WsOutputs(String p_name, OutputConfig p_config) {
        m_name = p_name;
        m_config = p_config;
    }

    /**
     * Returns the name mapped to the Output.
     * @return Name mapped to the Output.
     */
    public String getName() {
        return m_name;
    }

    /**
     * Returns the config of Output for the enumeration.
     * @return OutputConfig of enumeration.
     */
    public OutputConfig getConfig() {
        return m_config;
    }

    /**
     * Returns the actual Output object from the OutputManager
     * @return The corresponding output.
     */
    public Output get() {
        return Core.getOutputManager().getOutput(this);
    }
}