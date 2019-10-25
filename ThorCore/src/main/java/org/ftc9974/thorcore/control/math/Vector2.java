package org.ftc9974.thorcore.control.math;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.ftc9974.thorcore.util.MathUtilities;

import java.util.Locale;

/**
 * Utility class for doing 2D vector math.
 * Implements Euclidean vector operations in a regular Cartesian coordinate system
 * (+x -> right, +y -> up, heading of 0 -> right, heading increases counterclockwise)
 */
@SuppressWarnings({"WeakerAccess", "unused"})
public final class Vector2 {

    public static final Vector2 ZERO = new Vector2(0, 0);

    private double values[];

    public Vector2(double x, double y) {
        values = new double[]{x, y};
    }

    public double get(int axis) {
        return values[axis];
    }

    public double getX() {
        return get(0);
    }

    public double getY() {
        return get(1);
    }

    public void set(int axis, double value) {
        values[axis] = value;
    }

    public void setX(double value) {
        set(0, value);
    }

    public void setY(double value) {
        set(1, value);
    }

    public double getHeading() {
        return Math.atan2(getY(), getX());
    }

    public double getMagnitude() {
        return Math.hypot(getX(), getY());
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Vector2) {
            Vector2 other = (Vector2) obj;
            return other.getX() == getX() && other.getY() == getY();
        }
        return false;
    }

    @Override
    public String toString() {
        return String.format(Locale.getDefault(), "(%f, %f)", values[0], values[1]);
    }

    // IMPORTANT:
    // operations are non-mutating!
    // that is, these methods will not change the vector object
    // that calls them!

    public Vector2 add(Vector2 other) {
        return add(this, other);
    }

    public Vector2 subtract(Vector2 other) {
        return subtract(this, other);
    }

    public Vector2 scalarMultiply(double scalar) {
        return scalarMultiply(this, scalar);
    }

    public Vector2 scalarDivide(double scalar) {
        return scalarDivide(this, scalar);
    }

    public Vector2 rotate(double theta) {
        return rotate(this, theta);
    }

    public static Vector2 add(Vector2 vec1, Vector2 vec2) {
        return new Vector2(vec1.getX() + vec2.getX(), vec1.getY() + vec2.getY());
    }

    public static Vector2 subtract(Vector2 vec1, Vector2 vec2) {
        return new Vector2(vec1.getX() - vec2.getX(), vec1.getY() - vec2.getY());
    }

    public static Vector2 scalarMultiply(Vector2 vec, double scalar) {
        return new Vector2(vec.getX() * scalar, vec.getY() * scalar);
    }

    public static Vector2 scalarDivide(Vector2 vec, double scalar) {
        return new Vector2(vec.getX() / scalar, vec.getY() / scalar);
    }

    public static Vector2 rotate(Vector2 vec1, double theta) {
        double[] rotated = MathUtilities.rotate2D(vec1.values, theta);
        return new Vector2(rotated[0], rotated[1]);
    }
}
