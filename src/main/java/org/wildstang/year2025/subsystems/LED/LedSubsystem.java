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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LedSubsystem implements Subsystem {

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private WsVision vision;
    private Timer timer =  new Timer();
    private Timer clock1 = new Timer();
    private Timer clock2 = new Timer();
    

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
    
    private int colorFn;
    private int startRand = 1;
    private int startRandG = 1;
    private int changeSpeed = 12;
    private int plusOr;
    private int flashColor = 2;
    private int flashHalf = 1;
    private int flashSpeedOne = 10;
    private int flashSpeedTwo = 12;
    private int currentColor = 0;
    private DigitalInput leftStick;
    private DigitalInput rightStick;
    private DigitalInput dpadUp;
    private int k = 0;
    private int c = 0;
    private int s = 0;
    private int shiftSpeed = 20;

    XboxController controller = new XboxController(0);

    @Override
    public void inputUpdate(Input source) {
        // TODO Auto-generated method stub
        // if (leftStick.getValue() == true){
        //     NormalBlue();
        // }
        
        // else if (rightStick.getValue() == true){
        //     k = 0;
        //     c = 0;
        //     colorFn = 1;
        //     if (flashColor == 1){
        //         flashColor = 2;
        //     }
        //     else if (flashColor == 2){
        //         flashColor = 1;
        //     }
        //     timer.stop();
        //     timer.reset();
        //     timer.start();
        //     controller.setRumble(RumbleType.kBothRumble, 0.5);
        // }
        // else if (dpadUp.getValue() == true){
        //     setGreen();
        // }
    }

    public void NormalBlue(){
        colorFn = 0;
        clock1.start();
            if(clock1.hasElapsed(0.05)){
                for(int i = 0; i < length; i++){
                    int randomNum = (int)(Math.random()*2);
                    switch(randomNum){
                    case 0:
                        ledBuffer.setRGB(i, 0, 0, 153);
                        break;
                    case 1:
                        ledBuffer.setRGB(i, 102, 178, 150);
                        break;
                    case 2:
                        ledBuffer.setRGB(i, 200, 153, 200);
                        break;
                    }
                }
                clock1.stop();
                clock1.reset();
            }
        led.setData(ledBuffer);
    }

    public void setGreen(){
        colorFn = 2;
        for(int i = 0; i < length; i++){
            ledBuffer.setRGB(i, 61,255,179);
        }
        led.setData(ledBuffer);
    }

    public void setBlack(){
        for(int i = 0; i < length; i++){
            ledBuffer.setRGB(i, 0, 0, 0);
        }
        led.setData(ledBuffer);
    }

    public void NormalGreen(){
        
        clock2.start();
            if(clock2.hasElapsed(0.05)){
                for(int i = 0; i < length; i++){
                    int randomNum = (int)(Math.random()*2);
                    switch(randomNum){
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
                clock2.stop();
                clock2.reset();
            }
        led.setData(ledBuffer);
    }
    public void Flash(){
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
                        ledBuffer.setRGB(i, (int)(0.5 * currentColor), currentColor, (int)(0.875 * currentColor));
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
                    }
                    if (flashColor == 2){
                        currentColor = (int)((Math.random() -1.5) * flashSpeedTwo) + ledBuffer.getGreen(i);
                        if (currentColor < 0){
                            currentColor = 0;
                        }
                        ledBuffer.setRGB(i, (int)(0.5 * currentColor), currentColor, (int)(0.675 * currentColor));
                    }
                }
            }
            else {
                flashHalf = 1;
                NormalBlue();
            }
        }
        led.setData(ledBuffer);
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
        dpadUp = (DigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_DPAD_UP);
        dpadUp.addInputListener(this); 

        }

    @Override
    public void selfTest() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'selfTest'");
    }

    @Override
    public void update() {
        if(timer.hasElapsed(0.65)){
            controller.setRumble(RumbleType.kBothRumble, 0);
            timer.stop();
            timer.reset();
            
        }
        // TODO Auto-generated method stub
        if (colorFn == 0){ //Standard coloring
            NormalBlue();
        }
        else if (colorFn == 1){ //Intake State, maybe outtake with red
            Flash();
        }
        else if (colorFn == 2){ //Vision detects algae
            setGreen();
        }

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