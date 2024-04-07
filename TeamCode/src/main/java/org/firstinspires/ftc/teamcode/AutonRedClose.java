package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.ArmPositions;
import org.firstinspires.ftc.teamcode.hardware.ArmPotentiometerPositions;
import org.firstinspires.ftc.teamcode.hardware.FlipperMotorPositions;
//import org.firstinspires.ftc.teamcode.hardware.GripperPositions;
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
                    lights.switchLight(Light.LED1, LightMode.RED);
                } else if (parkingPosition == 2) {
                    lights.switchLight(Light.LED1, LightMode.YELLOW);
                } else if (parkingPosition == 3) {
                    lights.switchLight(Light.LED1, LightMode.GREEN);
                } else if (parkingPosition == 0) {
                    lights.switchLight(Light.LED1, LightMode.OFF);
                }
                lastParkingPosition = parkingPosition;
            }
        }

        waitForStart();

        //Set the arm position up to not drag
        //armMotor.moveArmEncoded(ArmPositions.FRONT_ARC_ZERO);
        robotControlArmPotentiometer.moveToPosition(ArmPotentiometerPositions.DRIVE,armMotor,0.6);
        sleep(1000);


        //Left
        if(parkingPosition == 1){
            imuDrive(autonSlow, 28.5, 0);
            imuTurn(autonSlow, -90);
            imuDrive(autonSlow, 7, 0);
            imuDrive(autonSlow, -3, 0);

            //move arm down to deliver
            deliverSpikePixel();

            //drive towards pixel
            imuDrive(autonMedium, -20, 0);
            encoderStrafe(autonMedium,7.5,5);
            imuDrive(autonMedium, -15.5, 0);

            //deposit back pixel
            deliverBackdropPixel();

            //slide out of the way
            imuDrive(autonMedium,3,0);
            encoderStrafe(autonMedium,-35,5);
        }
        //Middle
        else if(parkingPosition == 2){
            imuDrive(autonSlow, 36, 0);
            //sleep(750);
            imuDrive(autonSlow, -5, 0);

            //move arm down to deliver
            deliverSpikePixel();

            //drive to deliver pixel
            imuDrive(autonMedium, -8, 0);
            imuTurn(autonMedium, -90);
            imuDrive(autonMedium, -10, 0);

            encoderStrafe(autonMedium,7,5);
            imuDrive(autonMedium, -23, 0);

            //deposit back pixel
            deliverBackdropPixel();

            //part out of the way
            imuDrive(autonMedium,3,0);
            encoderStrafe(autonMedium,-29,5);
        }
        //Right
        else if(parkingPosition == 3){
            imuDrive(autonSlow, 5, 0);
            encoderStrafe(autonSlow,13.5,5);
            //imuTurn(0.4, 61);
            imuDrive(autonFast, 25, 0);
            imuDrive(autonSlow,-6,0);

            //move arm down to deliver
            deliverSpikePixel();

            //drive to deliver pixel
            imuDrive(autonMedium, -4, 0);
            encoderStrafe(autonMedium, 26.5, 5);
            imuTurn(autonMedium, -90);

            //deliver pixel
            deliverBackdropPixel();

            //park
            imuDrive(autonMedium,3,0);
            encoderStrafe(autonMedium, -15, 5);
            sleep(250);
        }
        //Error unable to find target so slide to backdrop
        else{
            imuDrive(.5, 5, 0);
            encoderStrafe(0.5, 47, 5);
            telemetry.addData("Park Position Unknown",parkingPosition);
        }

        //set arm down in the end
        armMotor.moveArmEncoded(ArmPositions.FRONT_ARC_MIN);
    }
}

