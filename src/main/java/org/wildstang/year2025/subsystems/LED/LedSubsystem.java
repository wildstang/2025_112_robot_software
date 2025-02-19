package org.wildstang.year2025.subsystems.LED;

import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.year2025.subsystems.targeting.WsVision;

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
    private int flashColor = 0;
    private int flashHalf = 1;
    private int flashSpeedOne = 7;
    private int flashSpeedTwo = 4;
    private int currentColor = 0;

    @Override
    public void inputUpdate(Input source) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'inputUpdate'");
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
                    int randBlue = (int)(Math.random() * 255);
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
                    if (currentBlue < 100){
                        currentBlue = currentBlue + changeSpeed / 2;
                    }
                    ledBuffer.setRGB(i, 0, 0, currentBlue);
                }
            }
        }

        if (doFlash == 1){
            doRand = 0;
            if (flashHalf == 1){
                for (int k = 0; k < 255 / (1 + flashSpeedOne); k++){
                    for (int i = 0; i < length; i++){
                        currentColor = (int)((Math.random() + 1) * flashSpeedOne) + currentColor;
                        if (currentColor > 255){
                            currentColor = 255;
                            if (flashColor == 1){
                                ledBuffer.setRGB(i, currentColor, 0, ledBuffer.getBlue(i));
                            }
                            if (flashColor == 2){
                                ledBuffer.setRGB(i, 0, currentColor, ledBuffer.getBlue(i));
                            }
                        }    
                    }
                }
                flashHalf = 2;
            }
            else if (flashHalf == 2){
                for (int k = 0; k < 255 / (0.5 * flashSpeedTwo); k++){
                    for (int i = 0; i < length; i++){
                        currentColor = (int)((Math.random() -1.5) * flashSpeedTwo) + currentColor;
                        if (currentColor < 0){
                            currentColor = 0;
                            if (flashColor == 1){
                                ledBuffer.setRGB(i, currentColor, 0, ledBuffer.getBlue(i));
                            }
                            if (flashColor == 2){
                                ledBuffer.setRGB(i, 0, currentColor, ledBuffer.getBlue(i));
                            }
                        }
                    }
                }
                doFlash = 0;
                doRand = 1;
                flashHalf = 1;
            }
        }
    }

    @Override
    public void resetState() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetState'");
    }

    @Override
    public void initSubsystems() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'initSubsystems'");
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getName'");
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