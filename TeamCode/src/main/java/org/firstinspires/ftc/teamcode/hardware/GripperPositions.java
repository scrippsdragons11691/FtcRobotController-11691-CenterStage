package org.firstinspires.ftc.teamcode.hardware;

public enum GripperPositions {

    UNKNOWN("UNKNOWN", 0),
    GRIPPER1_OPEN("GRIPPER1_OPEN", 0.9),
    GRIPPER1_CLOSED("GRIPPER1_CLOSED", 0.75),
    GRIPPER2_OPEN("GRIPPER2_OPEN", 0.1),
    GRIPPER2_CLOSED("GRIPPER2_CLOSED", 0.03),
    DRONE_READY("DRONE_READY",0.99),
    DRONE_LAUNCH("DRONE_LAUNCH", 0.50);


    private final String position;
    private final double servoPos;

    GripperPositions(String position, double servoPos){
        this.position = position;
        this.servoPos = servoPos;
    }

    public String getPosition(){
        return position;
    }

    public double getServoPos(){
        return servoPos;
    }
}
