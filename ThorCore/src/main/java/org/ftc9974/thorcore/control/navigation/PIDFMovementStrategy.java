package org.ftc9974.thorcore.control.navigation;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.ftc9974.thorcore.control.HolonomicDrivetrain;
import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.control.navigation.MovementStrategy;
import org.ftc9974.thorcore.util.MathUtilities;

public final class PIDFMovementStrategy implements MovementStrategy {

    private PIDF xPid, yPid, thetaPid;

    private double lastTargetX, lastTargetY, lastCurrentX, lastCurrentY, lastTargetT, lastCurrentT;
    private double xT, yT, tT;

    public PIDFMovementStrategy(double xKp, double xKi, double xKd, double xKf, double yKp, double yKi, double yKd, double yKf, double tKp, double tKi, double tKd, double tKf, double xDb, double yDb, double tDb, double xTh, double yTh, double tTh) {
        xPid = new PIDF(xKp, xKi, xKd, xKf);
        yPid = new PIDF(yKp, yKi, yKd, yKf);
        thetaPid = new PIDF(tKp, tKi, tKd, tKf);
        xPid.setOutputDeadband(xDb);
        yPid.setOutputDeadband(yDb);
        thetaPid.setOutputDeadband(tDb);
        xPid.setNominalOutputForward(0.2);
        xPid.setNominalOutputReverse(-0.2);
        xPid.setPeakOutputForward(1);
        xPid.setPeakOutputReverse(-1);
        yPid.setNominalOutputForward(0.2);
        yPid.setNominalOutputReverse(-0.2);
        yPid.setPeakOutputForward(1);
        yPid.setPeakOutputReverse(-1);
        thetaPid.setNominalOutputForward(0.2);
        thetaPid.setNominalOutputReverse(-0.2);
        thetaPid.setPeakOutputForward(1);
        thetaPid.setPeakOutputReverse(-1);
        xPid.setAtTargetThreshold(xTh);
        yPid.setAtTargetThreshold(yTh);
        thetaPid.setAtTargetThreshold(tTh);
        xT = xTh;
        yT = yTh;
        tT = tTh;
    }

    public PIDFMovementStrategy(PIDFCoefficients xCoefficients, PIDFCoefficients yCoefficients, PIDFCoefficients thetaCoefficients, double xDb, double yDb, double tDb, double xTh, double yTh, double tTh) {
        xPid = new PIDF(xCoefficients);
        yPid = new PIDF(yCoefficients);
        thetaPid = new PIDF(thetaCoefficients);
        xPid.setOutputDeadband(xDb);
        yPid.setOutputDeadband(yDb);
        thetaPid.setOutputDeadband(tDb);
        xPid.setNominalOutputForward(xDb);
        xPid.setNominalOutputReverse(-xDb);
        xPid.setPeakOutputForward(1);
        xPid.setPeakOutputReverse(-1);
        yPid.setNominalOutputForward(yDb);
        yPid.setNominalOutputReverse(-yDb);
        yPid.setPeakOutputForward(1);
        yPid.setPeakOutputReverse(-1);
        thetaPid.setNominalOutputForward(tDb);
        thetaPid.setNominalOutputReverse(-tDb);
        thetaPid.setPeakOutputForward(1);
        thetaPid.setPeakOutputReverse(-1);
        xPid.setAtTargetThreshold(xTh);
        yPid.setAtTargetThreshold(yTh);
        thetaPid.setAtTargetThreshold(tTh);
        xT = xTh;
        yT = yTh;
        tT = tTh;
    }

    @Override
    public double[] calculateMovement(HolonomicDrivetrain drivetrain, Vector2 currentPosition, double currentHeading, Vector2 targetPosition, double targetHeading) {
        lastTargetX = targetPosition.getX();
        lastTargetY = targetPosition.getY();
        lastCurrentX = currentPosition.getX();
        lastCurrentY = currentPosition.getY();
        lastTargetT = targetHeading;
        lastCurrentT = currentHeading;
        if (lastTargetX != xPid.getSetpoint()) {
            xPid.setSetpoint(lastTargetX);
        }
        if (lastTargetY != yPid.getSetpoint()) {
            yPid.setSetpoint(lastTargetY);
        }
        if (targetHeading != thetaPid.getSetpoint()) {
            thetaPid.setSetpoint(targetHeading);
        }
        double x = xPid.update(lastCurrentX);
        double y = yPid.update(lastCurrentY);
        double rot = thetaPid.update(currentHeading);
        double[] rotatedXY = MathUtilities.rotate2D(new double[] {x, y}, -currentHeading);
        return new double[] {rotatedXY[0], rotatedXY[0], rot};
    }

    @Override
    public boolean atPositionalTarget() {
        return Math.abs(lastTargetX - lastCurrentX) < xT && Math.abs(lastTargetY - lastCurrentY) < yT;
    }

    @Override
    public boolean atHeadingTarget() {
        return thetaPid.atTarget();
    }

    @Override
    public void reset() {
        xPid.resetControl();
        yPid.resetControl();
        thetaPid.resetControl();
    }

    public void setContinuity(double low, double high) {
        thetaPid.setContinuous(true);
        thetaPid.setContinuityRange(low, high);
    }
}
