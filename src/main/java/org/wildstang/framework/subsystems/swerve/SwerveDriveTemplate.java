package org.wildstang.framework.subsystems.swerve;

import org.wildstang.framework.subsystems.Subsystem;

public abstract class SwerveDriveTemplate implements Subsystem{

    public abstract void setAutoHeading(double headingTarget);

    public abstract void setGyro(double radians);

    public abstract void setToAuto();

    public abstract void setToTeleop();

    public abstract void setAutoValues(double velocityX, double velocityY, double angVel, double xPos, double yPos, double heading);

}
