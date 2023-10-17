package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.RobotHardwareMap;

public class RobotControlLights {

    RobotHardwareMap theHardwareMap;
    LinearOpMode opMode;

    static final String TAG = "RobotControlLights";
    int sequenceCounter = 0;
    LightSequence currentSequence = LightSequence.NONE;

    Light currentLights = Light.ALL;
    LightMode currentMode = LightMode.OFF;
    LightMode currentLightStep = LightMode.OFF;

    public enum LightSequence {
        NONE, ALTERNATE_GREEN, ALTERNATE_RED
    }

    public RobotControlLights(RobotHardwareMap theHardwareMap, LinearOpMode opMode) {
        this.opMode = opMode;
        this.theHardwareMap = theHardwareMap;
    }

    public void initialize(){
        //Log.d(TAG, "Lights init");

        theHardwareMap.LED1Green.setMode(DigitalChannel.Mode.OUTPUT);
        theHardwareMap.LED1Red.setMode(DigitalChannel.Mode.OUTPUT);
        theHardwareMap.LED2Green.setMode(DigitalChannel.Mode.OUTPUT);
        theHardwareMap.LED2Red.setMode(DigitalChannel.Mode.OUTPUT);
//        theHardwareMap.leftRedLED.setMode(DigitalChannel.Mode.OUTPUT);
//        theHardwareMap.leftGreenLED.setMode(DigitalChannel.Mode.OUTPUT);
//        theHardwareMap.frontRightRedLED.setMode(DigitalChannel.Mode.OUTPUT);
//        theHardwareMap.frontRightGreenLED.setMode(DigitalChannel.Mode.OUTPUT);
//        theHardwareMap.frontLeftRedLED.setMode(DigitalChannel.Mode.OUTPUT);
//        theHardwareMap.frontLeftGreenLED.setMode(DigitalChannel.Mode.OUTPUT);

        //runLightSeq(LightSequence.ALTERNATE_GREEN, 5);

        /*theHardwareMap.rightRedLED.setState(false);
        theHardwareMap.rightGreenLED.setState(true);

        theHardwareMap.leftRedLED.setState(false);
        theHardwareMap.leftGreenLED.setState(true);*/

    }

    /***
     * Used to switch on and off lights
     * @param light light to change mode
     * @param lightMode whether to turn on or off
     */
    public void switchLight( Light light, LightMode lightMode){
        boolean lightBoolGreen = true;
        boolean lightBoolRed = true;

        switch (lightMode){
            case OFF:
                lightBoolGreen = true;
                lightBoolRed = true;
                break;

            case RED:
                lightBoolGreen = false;
                lightBoolRed = true;
                break;

            case GREEN:
                lightBoolGreen = true;
                lightBoolRed = false;
                break;

            case YELLOW:
                lightBoolGreen = false;
                lightBoolRed = false;
                break;
        }

        switch (light){

            case LED1:
                theHardwareMap.LED1Green.setState(lightBoolGreen);
                theHardwareMap.LED1Red.setState(lightBoolRed);
                break;

            case LED2:
                theHardwareMap.LED2Green.setState(lightBoolGreen);
                theHardwareMap.LED2Red.setState(lightBoolRed);
                break;

            case ALL:
                theHardwareMap.LED1Green.setState(lightBoolGreen);
                theHardwareMap.LED1Red.setState(lightBoolRed);
                theHardwareMap.LED2Green.setState(lightBoolGreen);
                theHardwareMap.LED2Red.setState((lightBoolRed));
                break;
        }
    }

    /**
     * rotate between off, red, yellow, green, off...
     * @param changedLight
     */
    public void doLightSteps(Light changedLight) {
        LightMode newLightStep = LightMode.RED;

        if (currentLightStep == LightMode.OFF) {
            newLightStep = LightMode.RED;
        } else if (currentLightStep == LightMode.RED) {
            newLightStep = LightMode.YELLOW;
        } else if (currentLightStep == LightMode.YELLOW){
            newLightStep = LightMode.GREEN;
        } else if (currentLightStep == LightMode.GREEN){
            newLightStep = LightMode.OFF;
        }

        switchLight(changedLight, newLightStep);
        currentLightStep = newLightStep;
    }

   /* public void switchLights(){
        if (theHardwareMap.leftRedLED.getState()){
            theHardwareMap.leftRedLED.setState(false);
        } else {
            theHardwareMap.leftRedLED.setState(true);
        }
    }*/

    public void runLightSeq(LightSequence lightSeq, int times){
        currentSequence = lightSeq;
        sequenceCounter = times;
    }

    private void mainLoop(){
        if (sequenceCounter > 0){
            //Log.d(TAG, "light seq:" + sequenceCounter);
            if (currentSequence == LightSequence.ALTERNATE_GREEN){
                if (sequenceCounter % 2 == 0) {

                } else if (sequenceCounter == 1){
                    theHardwareMap.LED1Green.setState(true);
                    theHardwareMap.LED1Red.setState(false);
                } else {
                    theHardwareMap.LED1Green.setState(false);
                    theHardwareMap.LED2Red.setState(false);
                }
            } else if (currentSequence == LightSequence.ALTERNATE_RED){
                if (sequenceCounter % 2 == 0) {
                    theHardwareMap.LED1Green.setState(false);
                    theHardwareMap.LED2Red.setState(true);
                } else if (sequenceCounter == 1){
                    theHardwareMap.LED1Green.setState(false);
                    theHardwareMap.LED1Red.setState(true);
                } else {
                    theHardwareMap.LED1Green.setState(false);
                    theHardwareMap.LED1Red.setState(false);
                }
            }
            sequenceCounter--;

        }

    }

}

