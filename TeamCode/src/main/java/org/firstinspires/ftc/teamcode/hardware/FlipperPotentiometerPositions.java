package org.firstinspires.ftc.teamcode.hardware;

public enum FlipperPotentiometerPositions {

    MIN_VOLTAGE("MIN_VOLTAGE", 0),
    CLAW1_DOWN("CLAW1_DOWN", 0),
    CLAW2_DOWN("CLAW2_DOWN", 0),
    CLAW1_PLACE("CLAW1_PLACE", 0),
    CLAW2_PLACE("CLAW2_PLACE", 0),
    MAX_VOLTAGE("MAX_VOLTAGE", 0);

    private final String position;
    private final int voltagePos;

    FlipperPotentiometerPositions(String position, int voltagePos){
        this.position = position;
        this.voltagePos = voltagePos;
    }

    public String getPosition(){
        return position;
    }

    public int getVoltagePos(){
        return voltagePos;
    }
}
