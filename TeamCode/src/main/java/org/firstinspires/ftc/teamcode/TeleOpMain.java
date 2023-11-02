package org.firstinspires.ftc.teamcode;

import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.hardware.RobotControlLights;
import org.firstinspires.ftc.teamcode.hardware.RobotControlMechanum;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.hardware.Light;
import org.firstinspires.ftc.teamcode.hardware.LightMode;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit;

import java.io.IOException;
import java.util.List;

@TeleOp
public class TeleOpMain extends LinearOpMode {

    static final String TAG = "TeleOp Main";

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware
        RobotHardwareMap theHardwareMap = new RobotHardwareMap(this.hardwareMap, this);
        theHardwareMap.initialize();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        RobotControlMechanum robotDrive = new RobotControlMechanum(theHardwareMap, this);
        robotDrive.initialize();

        RobotControlLights lights = new RobotControlLights(theHardwareMap, this);
        lights.initialize();

        //Initial servo positions
        theHardwareMap.servoClaw1.setPosition(0.8);
        theHardwareMap.servoClaw2.setPosition(0.075);

        // waitForStart();

        // do something in init mode?
        while (opModeInInit()) {
            telemetry.addData("Robot", "Initialized successfully. Ready to run?");
            telemetry.update();
        }

        telemetry.addData("Robot", "running teleop.. press (Y) For telemetry");
        telemetry.update();

        lights.switchLight(Light.ALL, LightMode.OFF);

        //Initialize remaining variables
        double loopTimeStart = 0;
        boolean slowMode = true;
        lights.switchLight(Light.LED2, LightMode.GREEN); //Initial light for slow speed
        boolean showTelemetry = false;

        //create some gamepads to look at debouncing
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        //Set front camera up for AcmeTag
        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setCamera(theHardwareMap.frontCamera)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        //Main Loop
        while (opModeIsActive()) {

            loopTimeStart = System.currentTimeMillis();

            //copy over the previous gamepads so we can compare what changed
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            //mechanum drive - left stick y is negative because the up is negative
            double drive = -1 * gamepad1.left_stick_y;
            double strafe = -1 * gamepad1.left_trigger + gamepad1.right_trigger;
            //double strafe = gamepad1.left_stick_x;
            double twist = gamepad1.right_stick_x;

            //Speed values for slow mode
            if (slowMode) {
                drive *= 0.5;
                strafe *= 0.5;
                twist *= 0.5;

            } else { // non slow mode is only 75% power
                drive *= 0.75;
                strafe *= 0.75;
                twist *= 0.75;
            }

            robotDrive.teleOpMechanum(drive, strafe, twist);

            //slow mode
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                slowMode = true;
                lights.switchLight(Light.LED2, LightMode.GREEN);
            } else if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                slowMode = false;
                lights.switchLight(Light.LED2, LightMode.YELLOW);
            }

            //Open/close claw1
            telemetry.addData("Claw1 Position",theHardwareMap.servoClaw1.getPosition());

            if (currentGamepad2.a)
            {
                theHardwareMap.servoClaw1.setPosition(0.8);
                telemetry.addData("Claw1 Open",theHardwareMap.servoClaw1.getPosition());
            }
            else if (!currentGamepad2.a & previousGamepad2.a)
            {
                theHardwareMap.servoClaw1.setPosition(0.75);
                telemetry.addData("Claw1 Close",theHardwareMap.servoClaw1.getPosition());
            }

            //Open/close claw2
            telemetry.addData("Claw2 Position",theHardwareMap.servoClaw2.getPosition());

            if (currentGamepad2.b)
            {
                theHardwareMap.servoClaw2.setPosition(0.06);
                telemetry.addData("Claw2 Open",theHardwareMap.servoClaw2.getPosition());
            }
            else if (!currentGamepad2.b & previousGamepad2.b)
            {
                theHardwareMap.servoClaw2.setPosition(0.03);
                telemetry.addData("Claw2 Close",theHardwareMap.servoClaw2.getPosition());
            }

            //telemetry.update();

            //Arm Up/Down
            if (currentGamepad2.left_stick_y != 0)
            {
                theHardwareMap.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                double leftStickY = -currentGamepad2.left_stick_y;
                double currentPosition = theHardwareMap.armMotor.getCurrentPosition();
                if (leftStickY > 0 && currentPosition < 400) {
                    Log.d(TAG, "moving up  " + currentPosition + " " + leftStickY);
                    theHardwareMap.armMotor.setPower(0.8 * leftStickY);
                } else if (leftStickY < 0 && currentPosition > 500){
                    Log.d(TAG, "moving up from other side  " + currentPosition + " " + leftStickY);
                    theHardwareMap.armMotor.setPower(0.8 * leftStickY);
                } else if (leftStickY < 0 && currentPosition < 400){
                    Log.d(TAG, "slowing down  " + currentPosition + " " + leftStickY);
                    theHardwareMap.armMotor.setPower(0.15);

                } else {
                    Log.d(TAG, "moving no power  " + currentPosition + " " + leftStickY);
                    theHardwareMap.armMotor.setPower(0);
                }
            } else {
                //float code
                double currentPosition = theHardwareMap.armMotor.getCurrentPosition();
                if (currentPosition < 150 && currentPosition > 10) {
                    theHardwareMap.armMotor.setPower(0.3);
                    Log.d(TAG, "floating... " + currentPosition);
                } else if (currentPosition > 500 && currentPosition < 800){
                    theHardwareMap.armMotor.setPower(-0.3);
                    Log.d(TAG, "negative floating..." + currentPosition);
                } else {
                    theHardwareMap.armMotor.setPower(0);
                    Log.d(TAG, "stopped... " + currentPosition);
                }

            }
            telemetry.addData("armMotor Position",theHardwareMap.armMotor.getCurrentPosition());
            telemetry.addData("armMotor current", theHardwareMap.armMotor.getCurrent(CurrentUnit.MILLIAMPS));

            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up){
                //-638 is maximum
                //-440 is the 90 degree angle
                theHardwareMap.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                theHardwareMap.armMotor.setTargetPosition(600);
                //theHardwareMap.armMotor.setPower(0.8);
                theHardwareMap.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            } else if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down){
                theHardwareMap.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                theHardwareMap.armMotor.setTargetPosition(90);
                //theHardwareMap.armMotor.setPower(0.8);
                theHardwareMap.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            } else if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right){
                theHardwareMap.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                theHardwareMap.armMotor.setTargetPosition(440);
                //theHardwareMap.armMotor.setPower();
                theHardwareMap.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            //Flipper
            if (currentGamepad2.right_stick_y !=0)
            {
                telemetry.addData("CLAW PRESSED! ", currentGamepad2.right_stick_y);
                theHardwareMap.clawRotator.setPower(0.4 * currentGamepad2.right_stick_y);
            }
            else
            {
                theHardwareMap.clawRotator.setPower((0));

            }
            telemetry.addData("Claw Rotator Position",theHardwareMap.clawRotator.getCurrentPosition());



            //telemetry.update();

            //Check for detections
            lights.switchLight(Light.LED1, LightMode.OFF);
            //telemetry.clear();

            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

            //loop through the AprilTag detections to see what we found
            for (AprilTagDetection detection : currentDetections){
                //update the lights that we found one
                lights.switchLight(Light.LED1, LightMode.GREEN);

                //If we found something, display the data for it
                if (detection.ftcPose != null) {
                    telemetry.addData("Tag Bearing", detection.ftcPose.bearing);
                    telemetry.addData("Tag Range", detection.ftcPose.range);
                    telemetry.addData("Tag Yaw", detection.ftcPose.yaw);
                    telemetry.addData("ID",detection.id);

                }
                //If we detect a specific apriltag and they are pressing X, then we are twisting to that angle
                if (detection.id == 5 && currentGamepad1.x) {
                    telemetry.addData("Driveauto",detection.id);
                    double twistAmount = 0.25;
                    //adjust the twist based on the amount of yaw
                    //tweak the color for 5 and or 2
                    //add support for finding 2
                    //test other buttons for ease of use

                    if (detection.ftcPose.yaw >= 0) {
                        twistAmount = twistAmount * -1;
                    }
                    robotDrive.teleOpMechanum(0,0, twistAmount);
                }
            }
            //telemetry.update();

            //show telemetry
            if (currentGamepad1.y && !previousGamepad1.y) {
                showTelemetry = !showTelemetry;
            }

            //if (showTelemetry) {
            if (1 == 0){
                telemetry.addData("Control1 Left X",currentGamepad1.left_stick_x);
                telemetry.addData("Control1 Left Y",currentGamepad1.left_stick_y);
                telemetry.addData("Control1 Right X",currentGamepad1.right_stick_x);
                telemetry.addData("Control Right Y",currentGamepad1.right_stick_y);
                telemetry.addData("Bumper Left",currentGamepad1.left_bumper);
                telemetry.addData("Bumper Right",currentGamepad1.right_bumper);
                telemetry.addData("Slow Mode",slowMode);
                //telemetry.addData("Camera State",visionPortal.getCameraState());
                //telemetry.addData("ch (mA)", theHardwareMap.controlHub.getCurrent(CurrentUnit.AMPS));
                //telemetry.addData("eh (mA)", theHardwareMap.expansionHub.getCurrent(CurrentUnit.AMPS));
                //telemetry.addData("ch temp", theHardwareMap.controlHub.getTemperature(TempUnit.FARENHEIT));
                //telemetry.addData("eh temp", theHardwareMap.expansionHub.getTemperature(TempUnit.FARENHEIT));
            }

            telemetry.addData("looptime", System.currentTimeMillis() - loopTimeStart);
            telemetry.update();
        }

        telemetry.addData("Status", "Stopped");
        telemetry.update();

    }
}