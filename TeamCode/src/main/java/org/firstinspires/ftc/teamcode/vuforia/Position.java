package org.firstinspires.ftc.teamcode.vuforia;

public class Position {


    public static final Position INVALID_POSITION;

    static {
        INVALID_POSITION = new Position();
        INVALID_POSITION.validPosition = false;
    }

    private boolean validPosition;

    private double x;
    private double y;

    public Position() {
    }

    public Position(double x, double y) {
        validPosition = true;
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public boolean isValidPosition() {
        return validPosition;
    }
}
