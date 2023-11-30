package org.firstinspires.ftc.teamcode.hardware;

public enum GripperPositions {

    UNKNOWN("UNKNOWN", 0),
    GRIPPER1_OPEN("GRIPPER1_OPEN", 0.89),
    GRIPPER1_CLOSED("GRIPPER1_CLOSED", 0.6),
    GRIPPER2_OPEN("GRIPPER2_OPEN", 0.15),
    GRIPPER2_CLOSED("GRIPPER2_CLOSED", 0.025),
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
