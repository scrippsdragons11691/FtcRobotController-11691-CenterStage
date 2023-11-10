package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.ArmPositions;
import org.firstinspires.ftc.teamcode.hardware.GripperPositions;
import org.firstinspires.ftc.teamcode.hardware.Light;
import org.firstinspires.ftc.teamcode.hardware.LightMode;

@Autonomous(name = "Auton Red Close To Backstage", group = "Autons")
public class AutonRedClose extends AutonBase {



    private int     parkingPosition= -1;
    private int     lastParkingPosition = -1;
    @Override
    public void runOpMode(){

        boolean left = false;
        boolean middle = false;
        boolean right = true;

        initialize();

        while (opModeInInit()) {
            //check that the camera is open and working
            if (robotCameraHandler.frontCameraIsFaulty.get() && robotCameraHandler.enableFrontCamera)
                telemetry.addData("Status", "Front camera fault...Please restart");
            else
                telemetry.addData("Status", "Front Camera initialized");

            try {
                parkingPosition = robotCameraHandler.spikeLocationDetectionPipeline.getSpikeLocation().get();
                telemetry.addData("Parking Position", parkingPosition);
            } catch (NullPointerException npe) {
                telemetry.addData("cameras", npe.getMessage());
            }
            telemetry.update();

            if (lastParkingPosition != parkingPosition) {
                if (parkingPosition == 1) {

                    lights.switchLight(Light.ALL_LEFT, LightMode.RED);
                } else if (parkingPosition == 2) {
                    lights.switchLight(Light.ALL_LEFT, LightMode.YELLOW);
                } else if (parkingPosition == 3) {
                    lights.switchLight(Light.ALL_LEFT, LightMode.GREEN);
                } else if (parkingPosition == 0) {
                    lights.switchLight(Light.ALL_LEFT, LightMode.OFF);
                }
            }
        }

        waitForStart();

        //Left
        if(parkingPosition == 1){
            imuDrive(.4, 29.5, 0);
            imuTurn(.3, -90);
            imuDrive(.25, 7, 0);
            imuDrive(.15, -3, 0);
            sleep(1000);
            imuDrive(.4, -20, 0);
            imuTurn(.25, -15);
            imuDrive(.25, -14.5, 0);
            imuTurn(.25, 15);
            imuDrive(.25, -15, 0);
            sleep(5000);
            imuDrive(.25, 5, 0);
            imuTurn(0.75, 90);
            imuDrive(.5, 16, 0);
            imuTurn(0.75, 90);
            imuDrive(0.5, 15, 0);
        }
        //Middle
        else if(parkingPosition == 2){
            // lift arm before you start driving
            imuDrive(.3, 36, 0);
            sleep(750);
            imuDrive(.15, -5, 0);
            sleep(250);
            imuDrive(.15, -4, 0);
            imuTurn(.75, -90);
            imuDrive(0.5, -40, 0);
            sleep(5000);
            encoderStrafe(0.5, 28, 5);
            imuDrive(0.5, -13, 0);
        }
        //Right
        else if(parkingPosition == 3){
            imuDrive(.4, 19.5, 0);
            imuTurn(0.4, 61);
            imuDrive(0.25, 7, 0);
            sleep(750);
            imuDrive(0.25, -5.5, 0);
            imuTurn(.5, -150);
            imuDrive(0.5, -37.5, 0);
            //replace delay by placing pixel on backdrop
            sleep(5000);
            encoderStrafe(0.5, 35                   , 5);
            imuDrive(0.5, -13, 0);
        }
        //Error
        else{
            imuDrive(.5, 36, 0);
            imuDrive(.15, 5, 0);
            imuDrive(.25, 5, 0);
            sleep(2000);
            imuDrive(.15, -4, 0);
            imuTurn(.75, -90);
            imuDrive(0.5, -40, 0);
            //replace delay by placing pixel on backdrop
            sleep(5000);
            encoderStrafe(0.5, 28, 5);
            imuDrive(0.5, -13, 0);
        }
    }
}

