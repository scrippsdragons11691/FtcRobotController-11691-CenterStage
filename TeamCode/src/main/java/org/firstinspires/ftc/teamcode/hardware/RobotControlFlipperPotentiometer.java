package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.RobotHardwareMap;
import java.lang.Math;

public class RobotControlFlipperPotentiometer {
    RobotHardwareMap robotHardwareMap;
    LinearOpMode opMode;
    String potentiometerLocation = "potentiometer";
    AnalogInput potentiometer;

    public RobotControlFlipperPotentiometer(RobotHardwareMap robotHardwareMap, LinearOpMode opMode, String potentiometerLocation){
        this.opMode = opMode;
        this.robotHardwareMap = robotHardwareMap;
        this.potentiometerLocation = potentiometerLocation;
        initialize();
    }

    public void initialize(){
        potentiometer = robotHardwareMap.baseHMap.get(AnalogInput.class, potentiometerLocation);
    }

    public void moveToPosition(FlipperPotentiometerPositions flipperPotentiometerPositions, RobotControlFlipperMotor flipperMotor, double power){
        Log.println(Log.INFO, "Flipper Encoder: ", String.valueOf(flipperMotor.getCurrentPosition()));
        Log.println(Log.INFO, "Pot: ", String.valueOf(potentiometer.getVoltage()));

        //Keep moving until we are within 0.05 volts of the target position
        while(Math.abs(getCurrentPotPosition()-flipperPotentiometerPositions.getVoltagePos()) > 0.05) {
            if (getCurrentPotPosition() > flipperPotentiometerPositions.getVoltagePos()) {
                flipperMotor.moveFlipperPowerPot(-power);
            } else if (getCurrentPotPosition() < flipperPotentiometerPositions.getVoltagePos()) {
                flipperMotor.moveFlipperPowerPot(power);
            }
        }

        //stop the motor
        flipperMotor.moveFlipperPowerPot(0);
    }

    public double getCurrentPotPosition(){
        return potentiometer.getVoltage();
    }
}
