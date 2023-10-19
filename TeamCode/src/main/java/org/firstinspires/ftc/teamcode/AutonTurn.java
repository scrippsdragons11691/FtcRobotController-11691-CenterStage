package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class AutonTurn extends BaseAutonIMU{

    static double   ANGLE_TOL           = 0.5;
    public boolean  is_moving           = false;
    private double  motorSetPoint       = 0;
    public double   actualangle;
    public double   error;
    ElapsedTime runtime;
    public double initialangle;
    public double   targetAngle;
    public double   targetSpeed;
    public double   targetDeltaSpeed;
    public double   timeout;
    double  power = .30, correction;
    Telemetry telemetry;

    public AutonTurn(RobotHardwareMap hMap) {
        super(hMap);

        runtime         = new ElapsedTime();
        initialangle     = getAbsoluteHeading();
        // telemetry=opMode.telemetry;

        // TODO: Determine motor direction
        robotHardwareMap.backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        robotHardwareMap.frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        robotHardwareMap.backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        robotHardwareMap.frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void AutonTurn_HighPowerAtEnd (double Angle, double speed, double deltaSpeed, double timeoute, LinearOpMode theOpMode) {
        is_moving=true;
        targetAngle = Angle;
        targetSpeed = speed;
        targetDeltaSpeed = deltaSpeed;
        timeout = timeoute;
        telemetry = theOpMode.telemetry;
        telemetry.addData("T_angle;",targetAngle);
        telemetry.update();
        gotoPosition_new(0.25,theOpMode);
    }

    public void gotoPosition_new(double powerForFinalAdjust, LinearOpMode theOpMode) {
        actualangle     = getAbsoluteHeading();

        error = targetAngle - actualangle;

        SetZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SetMotorsToRunWithoutEncoders();

        ElapsedTime rampTimer = new ElapsedTime();
        rampTimer.reset();
        double time = runtime.time();
        while((Math.abs(error) > ANGLE_TOL) && (runtime.time() - time < timeout)
                && !theOpMode.isStopRequested() && theOpMode.opModeIsActive())
        {
            actualangle = getAbsoluteHeading();
            error = targetAngle - actualangle;

            telemetry.addData("Target Angle","error= %.2f", error);
            telemetry.addData("Target Angle","value= %.2f", targetAngle);
            telemetry.addData("Actual Angle","value= %.2f", actualangle);
            telemetry.update();

            double rampedTargetSpeed = targetSpeed;
            double rampedTargetDeltaSpeed = targetDeltaSpeed;

            if(Math.abs(error) < RobotGlobalSettings.powerRampdownStartAngle )
            {
                rampedTargetSpeed = targetSpeed * (error / RobotGlobalSettings.powerRampdownStartAngle);
                rampedTargetSpeed = Range.clip(rampedTargetSpeed,RobotGlobalSettings.RotationRampDownMinimumPower,targetSpeed);

                if(Math.abs(targetDeltaSpeed) > 0.0001) {
                    rampedTargetDeltaSpeed = targetDeltaSpeed * (error / RobotGlobalSettings.powerRampdownStartAngle);
                    rampedTargetDeltaSpeed = Range.clip(rampedTargetDeltaSpeed, RobotGlobalSettings.RotationRampDownMinimumPower, targetDeltaSpeed);
                }
            }
            else if(rampTimer.seconds() <= RobotGlobalSettings.rotationRampTimeInSec) {
                rampedTargetSpeed = targetSpeed * (rampTimer.seconds() / RobotGlobalSettings.rotationRampTimeInSec);
                rampedTargetSpeed = Range.clip(rampedTargetSpeed,rampedTargetSpeed,targetSpeed);

                if(Math.abs(targetDeltaSpeed)> 0.0001) {
                    rampedTargetDeltaSpeed = targetDeltaSpeed * (rampTimer.seconds() / RobotGlobalSettings.rotationRampTimeInSec);
                    rampedTargetDeltaSpeed = Range.clip(rampedTargetDeltaSpeed, rampedTargetDeltaSpeed, targetDeltaSpeed);
                }
            }

            if (error < 0) {
                rotate(rampedTargetSpeed, rampedTargetDeltaSpeed); //clockwise
            } else if (error > 0) {
                rotate(-rampedTargetSpeed, -rampedTargetDeltaSpeed); //counterclockwise
            }
        }
        rotate(0,0);

    }

    void rotate(double motorSetPoint, double deltaPower){

        if(Math.abs(deltaPower) < 0.0001) {
            //todo maybe change the order
            robotHardwareMap.frontLeftMotor.setPower(motorSetPoint);
            robotHardwareMap.backLeftMotor.setPower(motorSetPoint);
            robotHardwareMap.frontRightMotor.setPower(-motorSetPoint);
            robotHardwareMap.backRightMotor.setPower(-motorSetPoint);
        }
        else {
            //todo maybe change the order
            robotHardwareMap.frontLeftMotor.setPower(motorSetPoint + deltaPower);
            robotHardwareMap.backLeftMotor.setPower(motorSetPoint + deltaPower);
            robotHardwareMap.frontRightMotor.setPower(motorSetPoint - deltaPower);
            robotHardwareMap.backRightMotor.setPower(motorSetPoint - deltaPower);
        }
    }

    public double getAbsoluteHeading() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
}

