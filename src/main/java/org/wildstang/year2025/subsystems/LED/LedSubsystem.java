package org.wildstang.year2025.subsystems.LED;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.year2025.robot.WsInputs;
import org.wildstang.year2025.subsystems.localization.WsVision;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class LedSubsystem implements Subsystem {

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private WsVision vision;
    private Timer timer =  new Timer();

    private int[] white = {255,255,255};
    private int[] blue = {0,0,255};
    private int[] red = {255,0,0};
    private int[] green = {0,255,0};
    private int[] orange = {255,100,0};
    private int[] cyan = {0,155,155};
    private int[] purple = {128,0,128};

    private int port = 0;//port
    private int length = 39;//length
    private int initialHue = 0;
    private int initialRed = 0;
    private int initialBlue = 0;
    
    private int doRand = 0;
    private int startRand = 1;
    private int changeSpeed = 2;
    private int plusOr;
    private int doFlash = 0;
    private int flashColor = 2;
    private int flashHalf = 1;
    private int flashSpeedOne = 5;
    private int flashSpeedTwo = 6;
    private int currentColor = 0;
    private DigitalInput leftStick;
    private DigitalInput rightStick;
    private int k = 0;
    private int c = 0;

    @Override
    public void inputUpdate(Input source) {
        // TODO Auto-generated method stub
        if (leftStick.getValue() == true){
            doRand = 1;
            doFlash = 0;
        }
        else if (rightStick.getValue() == true){
            doRand = 0;
            doFlash = 1;
            k = 0;
            c = 0;
            if (flashColor == 1){
                flashColor = 2;
            }
            else if (flashColor == 2){
                flashColor = 1;
            }
        }
    }

    @Override
    public void init() {
        // TODO Auto-generated method stub
        led = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(length);
        led.setLength(ledBuffer.getLength());
        setRGB(255, 255, 255);
        led.setData(ledBuffer);
        led.start();
        resetState();
        timer.start();
        for (int i = 0; i < length; i++){
            ledBuffer.setRGB(i, 0, 0, 0);
        }
        led.setData(ledBuffer);
        led.start();

        leftStick = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_JOYSTICK_BUTTON);
        leftStick.addInputListener(this);
        rightStick = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_JOYSTICK_BUTTON);
        rightStick.addInputListener(this);

        }

    @Override
    public void selfTest() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'selfTest'");
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
        if (doRand == 1){
            doFlash = 0;
            if (startRand == 1){ 
                startRand = 0;
                for (int i = 0; i < length; i++){
                    int randBlue = (int)((Math.random() * 155) + 100);
                    ledBuffer.setRGB(i, 0, 0, randBlue);
                }
            }
            else if (startRand == 0){
                for (int i = 0; i < length; i++){
                    double plusOrCheck = Math.random();
                    if (plusOrCheck >= 0.5){
                        plusOr = 1;
                    }
                    else if (plusOrCheck < 0.5){
                        plusOr = -1;
                    }
                    double numVal = Math.random() * changeSpeed * plusOr;
                    int currentBlue = (int)Math.abs(ledBuffer.getBlue(i) + numVal);
                    if (currentBlue > 255){
                        currentBlue = currentBlue - changeSpeed;
                    }
                    if (currentBlue < 50){
                        currentBlue = currentBlue + changeSpeed / 2;
                    }
                    ledBuffer.setRGB(i, 0, 0, currentBlue);
                }
            }
        }

        if (doFlash == 1){
            doRand = 0;
            if (flashHalf == 1){
                if (k <= 255 / (1 + flashSpeedOne)){
                    k++;
                    for (int i = 0; i < length; i++){
                        if (flashColor == 1){
                            currentColor = (int)((Math.random() + 1) * flashSpeedOne) + ledBuffer.getRed(i);
                            if (currentColor > 255){
                                currentColor = 255;
                            }
                            ledBuffer.setRGB(i, currentColor, 0, ((int)((ledBuffer.getBlue(i))/1.5)));
                        }
                        if (flashColor == 2){
                            currentColor = (int)((Math.random() + 1) * flashSpeedOne) + ledBuffer.getGreen(i);
                            if (currentColor > 255){
                                currentColor = 255;
                            }
                            ledBuffer.setRGB(i, 0, currentColor, ((int)((ledBuffer.getBlue(i))/1.5)));
                        }
                    }
                }
                else {
                    flashHalf = 2;
                }
            }
            else if (flashHalf == 2){
                if (c <= 255 / (0.5 * flashSpeedTwo)){
                    c++;
                    for (int i = 0; i < length; i++){
                            if (flashColor == 1){
                                currentColor = (int)((Math.random() -1.5) * flashSpeedTwo) + ledBuffer.getRed(i);
                                if (currentColor < 0){
                                    currentColor = 0;
                                }
                                ledBuffer.setRGB(i, currentColor, 0, ledBuffer.getBlue(i));
                            if (flashColor == 2){
                                currentColor = (int)((Math.random() -1.5) * flashSpeedTwo) + ledBuffer.getGreen(i);
                                if (currentColor < 0){
                                    currentColor = 0;
                                }
                                ledBuffer.setRGB(i, 0, currentColor, ledBuffer.getBlue(i));
                            }
                        }
                    }
                }
                else {
                    doFlash = 0;
                    doRand = 1;
                    flashHalf = 1;
                }
            }
        }
        led.setData(ledBuffer);
    }

    @Override
    public void resetState() {
        // TODO Auto-generated method stub
    }

    @Override
    public void initSubsystems() {
        // TODO Auto-generated method stub
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return ("LED");
    }

    public void setRGB(int red, int green, int blue){
        for (int i = 0; i < length; i++){
            ledBuffer.setRGB(i, (int)(0.75*red), (int)(0.75*green), (int)(0.75*blue));
        }
    }

    public void setRGB(int[] color){
        setRGB(color[0],color[1],color[2]);
    }
}