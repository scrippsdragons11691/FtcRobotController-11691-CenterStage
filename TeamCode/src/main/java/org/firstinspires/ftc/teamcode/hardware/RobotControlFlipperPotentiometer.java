package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.RobotHardwareMap;

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
        if (getCurrentPotPosition() > flipperPotentiometerPositions.getVoltagePos()){
            flipperMotor.moveFlipperPower(-power);
        }
        else if(getCurrentPotPosition() < flipperPotentiometerPositions.getVoltagePos()){
            flipperMotor.moveFlipperPower(power);
        }
        else{
            flipperMotor.moveFlipperPower(0);
        }
    }

    public double getCurrentPotPosition(){
        return potentiometer.getVoltage();
    }
}
