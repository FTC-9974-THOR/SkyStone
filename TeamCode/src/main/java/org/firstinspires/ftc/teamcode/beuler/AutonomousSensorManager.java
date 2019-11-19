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
    public ColorSensor tapeSensor,
                       skystoneSensor;

    @Hardware
    public DistanceSensor leftDistance,
                          rightDistance,
                          stoneSensor,
                          wallSensor0,
                          wallSensor1,
                          ultrasonic;

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

    /**
     * Get left distance, in cm
     * @return distance, in cm
     */
    public double getLeftDistance() {
        return leftDistance.getDistance(DistanceUnit.CM);
    }

    /**
     * Get right distance, in cm
     * @return distance, in cm
     */
    public double getRightDistance() {
        return rightDistance.getDistance(DistanceUnit.CM);
    }

    public double getStoneDistance() {
        return stoneSensor.getDistance(DistanceUnit.MM);
    }

    public double getWallDistance() {
        double distance = wallSensor0.getDistance(DistanceUnit.MM);
        if (Double.isNaN(distance)) {
            distance = wallSensor1.getDistance(DistanceUnit.MM);
        }
        return distance;
    }

    public double getWall0Distance() {
        return wallSensor0.getDistance(DistanceUnit.MM);
    }

    public double getWall1Distance() {
        return wallSensor1.getDistance(DistanceUnit.MM);
    }

    public boolean isStonePresent() {
        double distance = stoneSensor.getDistance(DistanceUnit.MM);
        return !Double.isNaN(distance) && distance < 200;
    }

    public boolean frontAtWall() {
        double distance = getWallDistance();
        return !Double.isNaN(distance) && distance < 75;
    }

    public double getUltrasonicDistance() {
        return ultrasonic.getDistance(DistanceUnit.MM);
    }

    public boolean isSkystone() {
        return MathUtilities.sum(skystoneSensor.red(), skystoneSensor.green(), skystoneSensor.blue()) < 10;
    }
}
