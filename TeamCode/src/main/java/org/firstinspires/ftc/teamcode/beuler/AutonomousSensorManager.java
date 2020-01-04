package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.util.MathUtilities;

public class AutonomousSensorManager {

    @Hardware
    public DistanceSensor leftLaser,
                          rightLaser,
                          frontLaser,
                          backLaser;

    @Hardware
    public AnalogInput ultrasonic;

    @Hardware
    public ColorSensor tapeSensor;

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
        return getTapeSensorHSV()[1] > 0.28;
    }

    /**
     * Get left distance, in mm
     * @return distance, in mm
     */
    public double getLeftDistance() {
        return leftLaser.getDistance(DistanceUnit.MM);
    }

    /**
     * Get right distance, in mm
     * @return distance, in mm
     */
    public double getRightDistance() {
        return rightLaser.getDistance(DistanceUnit.MM);
    }

    /*public double getStoneDistance() {
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

    public double getWallLaser0Distance() {
        return wallLaser0.getDistance(DistanceUnit.MM);
    }

    public double getWallLaser1Distance() {
        return wallLaser1.getDistance(DistanceUnit.MM);
    }

    public boolean isStonePresent() {
        double distance = stoneSensor.getDistance(DistanceUnit.MM);
        return !Double.isNaN(distance) && distance < 200;
    }

    public boolean frontAtWall() {
        double distance = getWallDistance();
        return !Double.isNaN(distance) && distance < 75;
    }*/

    public double getFrontDistance() {
        return frontLaser.getDistance(DistanceUnit.MM);
    }

    public double getBackDistance() {
        return backLaser.getDistance(DistanceUnit.MM);
    }

    public double getUltrasonicDistance() {
        return 5 * 1024 * (ultrasonic.getVoltage() / 3.3);
    }
}
