package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class AutonDrive extends BaseAutonIMU{

    boolean is_moving = false;

    double minimumMotorPower = 0;

    ElapsedTime runtime     = new ElapsedTime();
    ElapsedTime rampTimer   = new ElapsedTime();

    public AutonDrive(RobotHardwareMap hMap) {
        super(hMap);

        // TODO: Determine motor direction
        robotHardwareMap.backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        robotHardwareMap.frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        robotHardwareMap.backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        robotHardwareMap.frontRightMotor.setDirection(DcMotor.Direction.FORWARD);

    }

    public void Auton_Straff (double dist_Straff_In, double power, double timeoutS, LinearOpMode theOpMode){

        int newStraffLeftFTarget;
        int newStraffRightFTarget;
        int newStraffLeftBTarget;
        int newStraffRightBTarget;

        robotHardwareMap.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robotHardwareMap.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robotHardwareMap.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robotHardwareMap.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Reset the encoders
        robotHardwareMap.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardwareMap.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardwareMap.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardwareMap.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // TODO: Check if this is actually true... probably isn't
        dist_Straff_In *= RobotGlobalSettings.OneTileLength_inch/27;

        //Determine new target postition, and pass to motor controller
        //TODO: Determine if the 0.707 number is correct
        newStraffLeftFTarget  = (int)(-1*dist_Straff_In * RobotGlobalSettings.COUNTS_PER_INCH / 0.707);
        newStraffRightFTarget = (int)(dist_Straff_In * RobotGlobalSettings.COUNTS_PER_INCH / 0.707);
        newStraffLeftBTarget  = (int)(dist_Straff_In * RobotGlobalSettings.COUNTS_PER_INCH / 0.707);
        newStraffRightBTarget = (int)(-1*dist_Straff_In * RobotGlobalSettings.COUNTS_PER_INCH / 0.707);

        //Send the target position to the REV module
        robotHardwareMap.frontLeftMotor.setTargetPosition(newStraffLeftFTarget);
        robotHardwareMap.frontRightMotor.setTargetPosition(newStraffRightFTarget);
        robotHardwareMap.backLeftMotor.setTargetPosition(newStraffLeftBTarget);
        robotHardwareMap.backRightMotor.setTargetPosition(newStraffRightBTarget);


        //Set the motors to run to encoder mode
        robotHardwareMap.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotHardwareMap.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotHardwareMap.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotHardwareMap.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set the motor speed
        runtime.reset();
        setMotorPowerForLinearMove(power);

        runtime.reset();

        while ((runtime.seconds() < timeoutS)
                && (robotHardwareMap.frontLeftMotor.isBusy())
                && !theOpMode.isStopRequested() && theOpMode.opModeIsActive()){

            theOpMode.telemetry.addData("is_moving drive", is_moving);
            theOpMode.telemetry.addData("Front LEft encoder","position= %d", robotHardwareMap.frontLeftMotor.getCurrentPosition());
            theOpMode.telemetry.addData("Front Right encoder","position= %d", robotHardwareMap.frontRightMotor.getCurrentPosition());
            theOpMode.telemetry.addData("Back Left encoder","position= %d", robotHardwareMap.backLeftMotor.getCurrentPosition());
            theOpMode.telemetry.addData("Back Right encoder","position= %d", robotHardwareMap.backRightMotor.getCurrentPosition());
            theOpMode.telemetry.addData("Runtime",runtime.time());
            theOpMode.telemetry.addData("Timeout", timeoutS);
            theOpMode.telemetry.update();
        }

        setMotorPowerForLinearMove(0);

        robotHardwareMap.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotHardwareMap.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotHardwareMap.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotHardwareMap.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double correction=0;
    double DrivingAngle = 0;
    int newLeftFTarget;
    int remainingEncoderCountsAbsolute;
    int minRemainingEncoderCountsAbsolute;
    public void encoderDriveAuton(double distanceInches, double power, double timeoutT, LinearOpMode theOpMode, boolean rampDown, boolean brakeAtEnd, boolean incremental, boolean nearest90){

        double rampTimeInSec = RobotGlobalSettings.RampUpTime;

        double leftInches   = distanceInches;

        if( brakeAtEnd )
            SetZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE);
        else
            SetZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        if(incremental) {
            newLeftFTarget += (int) (leftInches * RobotGlobalSettings.COUNTS_PER_INCH);
        }
        else {
            newLeftFTarget = (int) (leftInches * RobotGlobalSettings.COUNTS_PER_INCH);
            rampTimer.reset();
            StopAndResetEncoders();
        }


        SetMotorsToRunWithoutEncoders();

        // DrivingAngle is the heading at which we want to drive
        DrivingAngle = nearest90 ? Math.round(globalAngle / 90) * 90 : globalAngle;

        double effectiveEncoderCountRampDownThreshold = RobotGlobalSettings.EncoderCountRampDownThreshold;

        minRemainingEncoderCountsAbsolute = 100000; // assume that we will never reach this high count

        int minEncoderValueLatch = 3;
        runtime.reset();
        int state = 0;
        while ((runtime.seconds() < timeoutT)
                && !theOpMode.isStopRequested() && theOpMode.opModeIsActive()) {

            double rampedPower = power;
            int remainingEncoderCounts = newLeftFTarget - robotHardwareMap.frontLeftMotor.getCurrentPosition();
            remainingEncoderCountsAbsolute = Math.abs(remainingEncoderCounts);

            if(remainingEncoderCountsAbsolute < minRemainingEncoderCountsAbsolute) {
                minRemainingEncoderCountsAbsolute = remainingEncoderCountsAbsolute;
                if(minEncoderValueLatch >0)
                    minEncoderValueLatch--;
            }


            if((remainingEncoderCountsAbsolute < 20) ||
                    ((minEncoderValueLatch == 0) && (remainingEncoderCountsAbsolute > minRemainingEncoderCountsAbsolute))) {
                break;
            }

            if ( rampDown && (remainingEncoderCountsAbsolute < effectiveEncoderCountRampDownThreshold)) {
                state = 2;
                // If we shut down motor power suddenly, the robot will slide. Therefore ramp power down
                rampedPower = Range.clip(power * (((double) remainingEncoderCountsAbsolute) / effectiveEncoderCountRampDownThreshold), RobotGlobalSettings.LinearRampDownMinimumPower, power);
            } else {
                state = 1;
                // The slower the robot speed is, delay the start of the power ramp down so that we do not waste time
                double rampDownStartModifier = ((DcMotorEx) robotHardwareMap.frontLeftMotor).getVelocity(AngleUnit.RADIANS) / RobotGlobalSettings.topWheelAngularVelocity_radPerSec;
                rampDownStartModifier = 1;
                effectiveEncoderCountRampDownThreshold = Math.abs(RobotGlobalSettings.EncoderCountRampDownThreshold * rampDownStartModifier);

                if (rampTimer.seconds() <= rampTimeInSec) {
                    // Spinning the wheels introduces an error when driving using encoders. Therefore ramp the wheel power up so that the wheels do not spin.
                    rampedPower = Range.clip(power * (rampTimer.seconds() / rampTimeInSec), minimumMotorPower, power);
                }
            }

            if( distanceInches < 0 )
                rampedPower *= -1;

            // Use gyro to drive in a straight line.
            correction = checkDirection(DrivingAngle);
            if(Math.abs(rampedPower) < 0.35)
                correction *= 1.15;

            setMotorPowerForLinearMove(rampedPower - correction, rampedPower + correction,
                    rampedPower + correction,rampedPower - correction);
        }

        if( rampDown || theOpMode.isStopRequested() || !theOpMode.opModeIsActive() ) {
            setMotorPowerForLinearMove(0);
        }
    }

    public void basicDrive (double power)    {

        robotHardwareMap.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotHardwareMap.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotHardwareMap.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotHardwareMap.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setMotorPowerForLinearMove(power);
    }

    double GlobalAngle;
    private double checkDirection(double angle)
    {
        double correction;

        GlobalAngle = getAngle();
        angle = angle - GlobalAngle;

        correction = angle * RobotGlobalSettings.ImuCorrectionFactor;

        return correction;
    }
}

