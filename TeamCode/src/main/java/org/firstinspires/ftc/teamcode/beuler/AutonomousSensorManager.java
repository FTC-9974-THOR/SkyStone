package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.util.MathUtilities;

public class AutonomousSensorManager {

    @Hardware
    public ColorSensor tapeSensor;

    @Hardware
    public DistanceSensor leftDistance, rightDistance;

    public AutonomousSensorManager(HardwareMap hw) {
        Realizer.realize(this, hw);
    }

    public double[] getTapeSensorRGB() {
        return new double[] {
                tapeSensor.red(),
                tapeSensor.green(),
                tapeSensor.blue()
        };
    }

    public double[] getTapeSensorHSV() {
        return MathUtilities.RGBtoHSV(
                tapeSensor.red() / 255.0,
                tapeSensor.green() / 255.0,
                tapeSensor.blue() / 255.0
        );
    }

    public boolean onTape() {
        return getTapeSensorHSV()[1] > 0.3;
    }

    public double getLeftDistance() {
        return leftDistance.getDistance(DistanceUnit.CM);
    }

    public double getRightDistance() {
        return rightDistance.getDistance(DistanceUnit.CM);
    }
}
