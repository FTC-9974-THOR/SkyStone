package org.ftc9974.thorcore.control.navigation;

import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.control.navigation.EncoderPositionCalculator;
import org.ftc9974.thorcore.util.MathUtilities;

public class MecanumEncoderCalculator implements EncoderPositionCalculator {

    private static final double RADICAL_SIN_QUARTER_PI = 1 / Math.sin(0.25 * Math.PI);
    private double[] COEFFICIENTS;

    public MecanumEncoderCalculator(double gearRatio) {
        COEFFICIENTS = new double[] {
            // 7 pulses per magnet *
            // 4 edges per pulse *
            // gear ratio
            (7 * 4 * gearRatio) / (MathUtilities.inchesToMM(4) * Math.PI),
            (7 * 4 * gearRatio) / (MathUtilities.inchesToMM(4) * Math.PI),
            (7 * 4 * gearRatio) / (MathUtilities.inchesToMM(4) * Math.PI),
            (7 * 4 * gearRatio) / (MathUtilities.inchesToMM(4) * Math.PI)
        };
    }

    @Override
    public int[] calculate(Vector2 point) {
        double mag = point.getMagnitude(), heading = (point.getHeading() + 0.5 * Math.PI) % (2 * Math.PI);
        int[] ret = new int[4];
        ret[0] = (int) (mag * COEFFICIENTS[0] * RADICAL_SIN_QUARTER_PI * Math.sin(heading - 0.25 * Math.PI));
        ret[1] = (int) (mag * COEFFICIENTS[1] * RADICAL_SIN_QUARTER_PI * Math.sin(heading - 0.75 * Math.PI));
        ret[2] = (int) (mag * COEFFICIENTS[2] * RADICAL_SIN_QUARTER_PI * Math.sin(heading - 0.75 * Math.PI));
        ret[3] = (int) (mag * COEFFICIENTS[3] * RADICAL_SIN_QUARTER_PI * Math.sin(heading - 0.25 * Math.PI));
        return ret;
    }
}
