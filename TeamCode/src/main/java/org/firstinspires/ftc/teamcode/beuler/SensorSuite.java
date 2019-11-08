package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.util.MathUtilities;

public class SensorSuite {

    @Hardware
    public ColorSensor tapeSensor;

    public SensorSuite(HardwareMap hw) {
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
        return getTapeSensorHSV()[1] > 0.5;
    }
}
