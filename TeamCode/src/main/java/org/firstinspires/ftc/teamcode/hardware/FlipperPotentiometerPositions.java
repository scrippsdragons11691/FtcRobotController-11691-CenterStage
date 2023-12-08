package org.firstinspires.ftc.teamcode.hardware;

public enum FlipperPotentiometerPositions {

    MIN_VOLTAGE("MIN_VOLTAGE", 0),
    CLAW1_DOWN("CLAW1_DOWN", 2.892),
    CLAW2_DOWN("CLAW2_DOWN", 0.455),
    CLAW1_PLACE("CLAW1_PLACE", 0),
    CLAW2_PLACE("CLAW2_PLACE", 1.858),
    MAX_VOLTAGE("MAX_VOLTAGE", 4);

    private final String position;
    private final double voltagePos;

    FlipperPotentiometerPositions(String position, double voltagePos){
        this.position = position;
        this.voltagePos = voltagePos;
    }

    public String getPosition(){
        return position;
    }

    public double getVoltagePos(){
        return voltagePos;
    }
}
