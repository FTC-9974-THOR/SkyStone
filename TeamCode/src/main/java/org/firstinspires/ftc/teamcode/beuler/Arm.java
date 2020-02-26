package org.firstinspires.ftc.teamcode.beuler;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.util.MathUtilities;

public class Arm {

    private static final double HIGH_SHOULDER_LIMIT = 3.25,
                                MID_SHOULDER = 1.39,
                                LOW_SHOULDER_LIMIT = 0.389,
                                JAW0_OPEN_POS = MathUtilities.map(1910, 500, 2500, 0, 1),
                                JAW0_PUSH_POS = MathUtilities.map(938, 500, 2500, 0, 1),
                                JAW0_CLOSED_POS = MathUtilities.map(700, 500, 2500, 0, 1),
                                JAW1_OPEN_POS = MathUtilities.map(800, 500, 2500, 0, 1),
                                JAW1_READY_POS = MathUtilities.map(1790, 500, 2500, 0, 1),
                                JAW1_PUSH_POS = MathUtilities.map(1698, 500, 2500, 0, 1),
                                JAW1_CLOSED_POS = MathUtilities.map(1870, 500, 2500, 0, 1),
                                RAMP_RATE = 0.5, // seconds to full power
                                LIFT_TOP_EXTENT = 5905; //8260 * (50.9 / 71.2);

    @Hardware
    public DcMotorEx shoulder, lift;

    @Hardware
    public ServoImplEx capstone, jaw0, jaw1;

    @Hardware
    public AnalogInput pot;

    @Hardware
    public TouchSensor homingSwitch, stowExtent;

    private PIDF shoulderPid;
    private boolean armClosedLoopEnabled;
    public double shoulderStartPoint;
    private boolean armInControlledMotion;
    public double speedLimit, toTarget, fromStart, pidOutput;

    private PIDF liftPid;
    private boolean liftClosedLoopEnabled;

    private boolean grabberOpen;

    private double lastShoulderPower;
    private long lastShoulderUpdateTime;

    private boolean liftHomed, lastHomingSwitchReading;
    private int liftEncoderOffset;

    public Arm(HardwareMap hw) {
        Realizer.realize(this, hw);
        jaw0.setPwmRange(new PwmControl.PwmRange(
                500, // closed
                2500 // open
        ));
        jaw1.setPwmRange(new PwmControl.PwmRange(
                500, // open
                2500  // closed
        ));
        capstone.setPwmRange(new PwmControl.PwmRange(
                950, // drop
                1700  // hold
        ));

        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //shoulder.setDirection(DcMotorSimple.Direction.REVERSE);

        shoulderPid = new PIDF(2, 0, 0, 0);
        shoulderPid.setNominalOutputForward(0.1);
        shoulderPid.setNominalOutputReverse(-0.1);
        shoulderPid.setPeakOutputForward(1);
        shoulderPid.setPeakOutputReverse(-1);
        shoulderPid.setAtTargetThreshold(0.01);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftPid = new PIDF(1.0/500.0, 0, 0, 0);
        liftPid.setNominalOutputReverse(0);
        liftPid.setNominalOutputForward(0.1);
        liftPid.setPeakOutputForward(1);
        liftPid.setPeakOutputReverse(-1);
        liftPid.setAtTargetThreshold(15);
    }

    void grab() {
        grabberOpen = false;
        jaw0.setPosition(JAW0_CLOSED_POS);
        jaw1.setPosition(JAW1_CLOSED_POS);
    }

    void release() {
        grabberOpen = true;
        jaw0.setPosition(JAW0_OPEN_POS);
        if (getArmPosition() < MID_SHOULDER) {
            jaw1.setPosition(JAW1_OPEN_POS);
        } else {
            jaw1.setPosition(JAW1_READY_POS);
        }
    }

    void configureForPush() {
        jaw1.setPosition(JAW1_PUSH_POS);
        jaw0.setPosition(JAW0_PUSH_POS);
    }

    void setJaw0Position(double position) {
        jaw0.setPosition(position);
    }

    void setJaw1Position(double position) {
        jaw1.setPosition(position);
    }

    void holdCapstone() {
        capstone.setPosition(1);
    }

    void releaseCapstone() {
        capstone.setPosition(0);
    }

    void stopShoulder() {
        shoulder.setPower(0);
        lastShoulderPower = 0;
    }

    void setShoulderPower(double power) {
        //if ((power < 0 && getArmPosition() < MID_SHOULDER) || (power > 0 && getArmPosition() > MID_SHOULDER)) {
        //    power = MathUtilities.constrain(power, -0.5, 0.5);
        //}

        long now = SystemClock.uptimeMillis();
        if (!armClosedLoopEnabled) {
            double deltaTime = (now - lastShoulderUpdateTime) / 1000.0;
            double input = lastShoulderPower - power;
            double rampFactor = deltaTime / RAMP_RATE;
            if (Math.abs(input) < rampFactor) {
                rampFactor = input;
            }
            lastShoulderPower -= Math.copySign(rampFactor, input);
            lastShoulderPower = MathUtilities.constrain(lastShoulderPower, -1, 1);
        } else {
            lastShoulderPower = power;
        }
        lastShoulderUpdateTime = now;

        double armPosition = getArmPosition();

        if (armPosition < LOW_SHOULDER_LIMIT) {
            lastShoulderPower = Math.max(0.2, lastShoulderPower);
        } else if (armPosition > HIGH_SHOULDER_LIMIT || stowExtent.isPressed()) {
            lastShoulderPower = Math.min(0, lastShoulderPower);
        }

        shoulder.setPower(lastShoulderPower);
    }

    double getArmPosition() {
        return pot.getVoltage();
    }

    double getArmAngle() {
        return MathUtilities.map(getArmPosition(), 0, 3.286, Math.toRadians(270 - 23.5 + 5), Math.toRadians(-23.5 + 5));
    }

    void setArmClosedLoopEnabled(boolean enabled) {
        armClosedLoopEnabled = enabled;
    }

    boolean isArmClosedLoopEnabled() {
        return armClosedLoopEnabled;
    }

    void setArmTargetPosition(double target) {
        shoulderPid.setSetpoint(target);
    }

    double getArmTargetPosition() {
        return shoulderPid.getSetpoint();
    }

    public void startControlledArmMotion(double target) {
        shoulderStartPoint = getArmPosition();
        armInControlledMotion = true;
        setArmTargetPosition(target);
        setArmClosedLoopEnabled(true);
    }

    public void stopControlledArmMotion() {
        armInControlledMotion = false;
        setArmClosedLoopEnabled(false);
    }

    public double lastArmPIDError() {
        return shoulderPid.getLastError();
    }

    public boolean armAtTarget() {
        return shoulderPid.atTarget();
    }

    public void setLiftPower(double power) {
        if (liftAtBottom()) {
            lift.setPower(Math.max(0, power));
        } else if (liftHomed && getLiftPosition() > LIFT_TOP_EXTENT) {
            lift.setPower(Math.min(0, power));
        } else {
            lift.setPower(power);
        }
    }

    public boolean liftAtBottom() {
        return homingSwitch.isPressed();
    }

    public int getLiftPosition() {
        return lift.getCurrentPosition() - liftEncoderOffset;
    }

    public double getLiftHeight() {
        return MathUtilities.map(Math.max(0, getLiftPosition()), 0, LIFT_TOP_EXTENT, 330, 800);
    }

    public double getLiftMaxPosition() {
        return LIFT_TOP_EXTENT;
    }

    public void setLiftTargetPosition(double position) {
        liftPid.setSetpoint(position);
    }

    public double getLiftTargetPosition() {
        return liftPid.getSetpoint();
    }

    public void setLiftClosedLoopEnabled(boolean enabled) {
        liftClosedLoopEnabled = enabled;
    }

    public boolean isLiftClosedLoopEnabled() {
        return liftClosedLoopEnabled;
    }

    public double lastLiftPIDError() {
        return liftPid.getLastError();
    }

    public boolean liftAtTarget() {
        return liftPid.atTarget();
    }

    public boolean isLiftHomed() {
        return liftHomed;
    }

    public double getManipulatorHeight() {
        return getLiftHeight() + 13 * 24 * Math.sin(getArmAngle()) - 100;
    }

    public double getManipulatorOffset() {
        return -13 * 24 * Math.cos(getArmAngle());
    }

    public void update() {
        if (armClosedLoopEnabled) {
            if (armInControlledMotion) {
                double currentPosition = getArmPosition();
                double cruiseSpeed = 1;
                double stopSpeed = 0.3;
                if (currentPosition > MID_SHOULDER && getArmTargetPosition() > MID_SHOULDER && shoulderStartPoint < getArmTargetPosition()) {
                    cruiseSpeed = 0.5;
                }

                // slowdown curve
                speedLimit = cruiseSpeed;
                toTarget = Math.abs(getArmTargetPosition() - currentPosition);
                if (toTarget < 0.2) {
                    speedLimit = stopSpeed;
                } else if (toTarget < 0.5) {
                    speedLimit = MathUtilities.map(toTarget, 0.2, 0.5, stopSpeed, cruiseSpeed);
                }

                // startup curve
                if (Math.abs(getArmTargetPosition() - shoulderStartPoint) > 0.5) {
                    fromStart = Math.abs(shoulderStartPoint - currentPosition);
                    if (fromStart < 0.5) {
                        speedLimit = Math.min(speedLimit, MathUtilities.map(fromStart, 0, 0.5, 0.4, cruiseSpeed));
                    }
                }
                shoulderPid.setPeakOutputForward(speedLimit);
                shoulderPid.setPeakOutputReverse(-speedLimit);
            }
            pidOutput = shoulderPid.update(getArmPosition());
            setShoulderPower(pidOutput);
        }
        if (liftClosedLoopEnabled && liftHomed) {
            setLiftPower(liftPid.update(getLiftPosition()));
        }
        if (grabberOpen) {
            if (getArmPosition() < MID_SHOULDER) {
                jaw1.setPosition(JAW1_OPEN_POS);
            } else {
                jaw1.setPosition(JAW1_READY_POS);
            }
        }

        if (!liftHomed) {
            boolean bottomedOut = liftAtBottom();
            if (!bottomedOut && lastHomingSwitchReading) {
                liftHomed = true;
                liftEncoderOffset = lift.getCurrentPosition();
            }
            lastHomingSwitchReading = bottomedOut;
        }
    }
}
