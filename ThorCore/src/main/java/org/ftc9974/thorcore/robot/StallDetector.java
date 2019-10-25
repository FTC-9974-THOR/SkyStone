package org.ftc9974.thorcore.robot;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.DcMotor;

public final class StallDetector {

    private DcMotor motor;
    private long lastTime;
    private int lastPosition;
    private double stallThreshold;

    public StallDetector(DcMotor motor, double threshold) {
        this.motor = motor;
        lastTime = SystemClock.uptimeMillis();
        lastPosition = motor.getCurrentPosition();
        stallThreshold = threshold;
    }

    public boolean isStalled() {
        boolean ret = false;
        if (Math.abs(motor.getPower()) > 0.05) {
             ret = stallThreshold > (motor.getCurrentPosition() - lastPosition) / (SystemClock.uptimeMillis() - lastTime);
        }
        lastTime = SystemClock.uptimeMillis();
        lastPosition = motor.getCurrentPosition();
        return ret;
    }
}
