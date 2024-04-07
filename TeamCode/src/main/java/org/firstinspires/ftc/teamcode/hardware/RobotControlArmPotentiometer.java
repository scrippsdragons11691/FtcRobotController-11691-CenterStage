package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.teamcode.RobotHardwareMap;
import java.lang.Math;

public class RobotControlArmPotentiometer {
    RobotHardwareMap robotHardwareMap;
    LinearOpMode opMode;
    String potentiometerLocation = "armpot";
    AnalogInput potentiometer;

    public RobotControlArmPotentiometer(RobotHardwareMap robotHardwareMap, LinearOpMode opMode, String potentiometerLocation){
        this.opMode = opMode;
        this.robotHardwareMap = robotHardwareMap;
        this.potentiometerLocation = potentiometerLocation;
        initialize();
    }

    public void initialize(){
        potentiometer = robotHardwareMap.baseHMap.get(AnalogInput.class, potentiometerLocation);
    }

    public void moveToPosition(ArmPotentiometerPositions armPotentiometerPositions, RobotControlArm armMotor, double power)
    {
        //Log.println(Log.INFO, "ARM Encoder: ", String.valueOf(armMotor.getCurrentPosition()));
        //Log.println(Log.INFO, "ARM Pot: ", String.valueOf(potentiometer.getVoltage()));

        //Keep moving until we are within 0.05 volts of the target position
        while(Math.abs(getCurrentPotPosition() - armPotentiometerPositions.getVoltagePos()) > 0.05) {
            if (getCurrentPotPosition() > armPotentiometerPositions.getVoltagePos()) {
                armMotor.moveArmPowerPot(-power);
            } else if (getCurrentPotPosition() < armPotentiometerPositions.getVoltagePos()) {
                armMotor.moveArmPowerPot(power);
            }
        }

        //stop the motor
        //armMotor.moveArmPowerPot(0);
        armMotor.stopArmPotWithHold();

    }

    public double getCurrentPotPosition(){
        return potentiometer.getVoltage();
    }
}
