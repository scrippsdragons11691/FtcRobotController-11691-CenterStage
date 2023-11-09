package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

public class ArmMovePowerCalculator {

    static final String TAG = "ArmMovePowerCalculator";

    public static double calculatePowerForMove(int currentPosition, int targetPosition){

        double downPower = 1;
        double upPower = 2;
        double desiredPower = 0;
        double deltaE = targetPosition - currentPosition; //if delta is negative we are going down, positive up
        deltaE /= ArmPositions.BACK_ARC_MAX.getEncodedPos(); //divide by max encoded positions

        // for going up
        if (deltaE >= 0.01){
            desiredPower = deltaE * upPower;
            if (desiredPower > 1)
                desiredPower = 1;
            else if (desiredPower < 0.01)
                desiredPower = 0.1;

            //need more power for under 500
            if (currentPosition < ArmPositions.FRONT_ARC_STRAIGHT.getEncodedPos()){
                upPower *= 2;
            }

            //going down
        } else if (deltaE < 0.01){
            /*if (currentPosition < ArmPositions.FRONT_ARC_STRAIGHT.getEncodedPos()){
                desiredPower = -0.1;
            }*/
            desiredPower = deltaE * downPower;
            if (desiredPower < -1) {
                desiredPower = -1;
            } else if (desiredPower > -0.01 ) {
                desiredPower = -0.1;
            }
        } else {
            desiredPower = -0.1;
        }

        //if (desiredPower != 0)
        Log.d(TAG, "deltaE " + deltaE + " desiredP " + desiredPower);
        return desiredPower;
    }
}

