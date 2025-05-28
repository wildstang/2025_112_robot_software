package org.wildstang.year2025.subsystems.LED;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;

import org.wildstang.year2025.robot.WsSubsystems;
import org.wildstang.year2025.subsystems.localization.Localization;
import org.wildstang.year2025.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LedSubsystem implements Subsystem {

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private Timer rumbleTimer =  new Timer();
    private Timer patternClock = new Timer();

    public static enum LEDstates {NORMAL, INTAKE, SHOOT, ALGAE_DETECT, L2, L3, DEFENSE};
    public LEDstates ledState = LEDstates.NORMAL;

    private int port = 0;  //port
    private int length = 39;  //length

    private int flashColor = 2;
    private int flashHalf = 1;
    private int flashSpeedOne = 10;
    private int flashSpeedTwo = 20;
    private int currentColor = 0;
    private int k = 0;
    private int initialHue = 0;

    XboxController controller = new XboxController(0);
    SwerveDrive drive;
    Localization loc;

    @Override
    public void init() {
        led = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(length);
        led.setLength(ledBuffer.getLength());
        resetState();
        setRGB(0, 0, 255);
        led.setData(ledBuffer);
        led.start();
        patternClock.start();
    }

    @Override
    public void initSubsystems() {
        drive = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        loc = (Localization) Core.getSubsystemManager().getSubsystem(WsSubsystems.LOCALIZATION);
    }

    @Override
    public void inputUpdate(Input source) {
    }

    @Override
    public void update() {
        if (rumbleTimer.isRunning() && rumbleTimer.hasElapsed(0.65)) {
            controller.setRumble(RumbleType.kBothRumble, 0);
            rumbleTimer.stop();
            rumbleTimer.reset();
            ledState = LEDstates.NORMAL;
        }
        
        // override all other signals during last 17 to last 15 seconds of the match
        // to provide end of match signal
        if (DriverStation.isFMSAttached() && DriverStation.getMatchTime() < 17 && DriverStation.getMatchTime() > 15) {
            rainbow();
            return;
        }

        switch (ledState) {  
            case INTAKE:
                rumbleTimer.start();
                controller.setRumble(RumbleType.kBothRumble, 0.5);
                normalGreen();
                break;
            case SHOOT:
                rumbleTimer.start();
                controller.setRumble(RumbleType.kBothRumble, 0.5);
                normalGreen();
                break;
            case ALGAE_DETECT:
                if (drive.algaeInView()) {
                    setRGB(61,255,179);
                } else {
                    setRGB(194, 0, 76);
                }
                led.setData(ledBuffer);
                break;
            case L2:
                setSectionColor(13, 25, new int[]{61,255,179});
                break;
            case L3:
                setSectionColor(26, 38, new int[]{61,255,179});
                break;
            case DEFENSE:
                setRGB(255, 0, 0);
                led.setData(ledBuffer);
                break;
            case NORMAL:
                normalBlue();
                break;
        }
        SmartDashboard.putString("LED state", ledState.toString());
    }

    public void setSectionColor(int start, int end, int[] color) {
        setRGB(0, 0, 0);
        for (int i = start; i <= end; i++) {
            ledBuffer.setRGB(i, color[0], color[1], color[2]);
        }
        led.setData(ledBuffer);
    }

    public void normalBlue(){
        if (patternClock.hasElapsed(0.1)) {
            for (int i = 1; i < length; i++) {
                int randomNum = (int) (Math.random() * 3);
                switch (randomNum) {
                    case 0:
                        ledBuffer.setRGB(i, 0, 0, 255);
                        break;
                    case 1:
                        ledBuffer.setRGB(i, 72, 241, 247);
                        break;
                    case 2:
                        ledBuffer.setRGB(i, 19, 53, 107);
                        break;
                }
            }
            patternClock.reset();
        }

        // PhotonVision diagnostic light
        if (loc.frontHasEst && loc.rearHasEst) ledBuffer.setRGB(0, 0, 255, 0);
        else if (loc.frontHasEst && !loc.rearHasEst) ledBuffer.setRGB(0, 0, 0, 255);
        else if (!loc.frontHasEst && loc.rearHasEst) ledBuffer.setRGB(0, 255, 0, 255);
        else ledBuffer.setRGB(0, 255, 0, 0);

        led.setData(ledBuffer);
    }

    private void rainbow() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setHSV(i, 180 - (initialHue + (i * 180 / ledBuffer.getLength())) % 180, 255, 128);
        }
        initialHue = (initialHue + 3) % 180;
        led.setData(ledBuffer);
    }

    public void setGreen() {
        setRGB(0, 0, 0);
        for (int i = 0; i < 13; i++) {
            ledBuffer.setRGB(i, 61,255,179);
        }
        led.setData(ledBuffer);
    }

    public void normalGreen() {
        if (patternClock.hasElapsed(0.05)) {
            for(int i = 0; i < length; i++){
                int randomNum = (int) (Math.random() * 3);
                switch (randomNum) {
                    case 0:
                        ledBuffer.setRGB(i, 0, 153, 0);
                        break;
                    case 1:
                        ledBuffer.setRGB(i, 102, 200, 150);
                        break;
                    case 2:
                        ledBuffer.setRGB(i, 200, 255, 200);
                        break;
                }
            }
            patternClock.reset();
        }
        led.setData(ledBuffer);
    }

    public void normalGreenSection(int start, int end) {
        if (patternClock.hasElapsed(0.05)) {
            for (int i = start; i < end; i++) {
                int randomNum = (int) (Math.random() * 3);
                switch (randomNum) {
                    case 0:
                        ledBuffer.setRGB(i, 0, 153, 0);
                        break;
                    case 1:
                        ledBuffer.setRGB(i, 102, 200, 150);
                        break;
                    case 2:
                        ledBuffer.setRGB(i, 200, 255, 200);
                        break;
                }
            }
            patternClock.reset();
        }
        led.setData(ledBuffer);
    }

    public void flash() {
        setRGB(0, 0, 0);
        k++;
        if (flashHalf == 1){
            if (k <= (255 / flashSpeedOne)) {
                for (int i = 0; i < 13; i++) {
                    if (flashColor == 1) {
                        currentColor = (int) ((Math.random() + 1) * flashSpeedOne) + ledBuffer.getRed(i);
                        if (currentColor > 255) currentColor = 255;
                        ledBuffer.setRGB(i, currentColor, 0, (int) ((ledBuffer.getBlue(i)) / 1.5));
                    } else if (flashColor == 2) {
                        currentColor = (int) ((Math.random() + 1) * flashSpeedOne) + ledBuffer.getGreen(i);
                        if (currentColor > 255) currentColor = 255;
                        ledBuffer.setRGB(i, (int) (0.5 * currentColor), currentColor, (int) (0.875 * currentColor));
                    }
                }
            } else {
                setSectionColor(0, 13, new int[]{127, 255, 223});
                flashHalf = 2;
                k = 0;
            }
        } else if (flashHalf == 2) {
            if (k <= (255 / flashSpeedTwo)) {
                for (int i = 0; i < 13; i++) {
                    if (flashColor == 1) {
                        currentColor = (int) (-(Math.random() + 1) * flashSpeedTwo) + ledBuffer.getRed(i);
                        if (currentColor < 0) currentColor = 0;
                        ledBuffer.setRGB(i, currentColor, 0, ledBuffer.getBlue(i));
                    } else if (flashColor == 2) {
                        currentColor = (int) (-(Math.random() + 1) * flashSpeedTwo) + ledBuffer.getGreen(i);
                        if (currentColor < 0) currentColor = 0;
                        ledBuffer.setRGB(i, (int) (0.5 * currentColor), currentColor, (int) (0.675 * currentColor));
                    }
                }
            } else {
                setSectionColor(0, 13, new int[]{0, 0, 0});
                flashHalf = 1;
                k = 0;
            }
        }
        led.setData(ledBuffer);
    }

    public void setRGB(int red, int green, int blue){
        for (int i = 0; i < length; i++) {
            ledBuffer.setRGB(i, red, green, blue);
        }
    }

    public void setRGB(int[] color) {
        setRGB(color[0], color[1], color[2]);
    }

    @Override
    public void selfTest() {
    }

    @Override
    public void resetState() {
    }

    @Override
    public String getName() {
        return ("LED");
    }
}