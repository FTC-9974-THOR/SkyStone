package org.firstinspires.ftc.teamcode;

public enum StonePosition {
    LEFT,
    CENTER,
    RIGHT,

    UNKNOWN;


    @Override
    public String toString() {
        switch (this) {
            case LEFT:
                return "Left";
            case CENTER:
                return "Center";
            case RIGHT:
                return "Right";
            default:
            case UNKNOWN:
                return "Unknown";
        }
    }
}
