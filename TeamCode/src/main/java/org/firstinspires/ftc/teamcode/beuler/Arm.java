package org.firstinspires.ftc.teamcode.beuler;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.robot.Motor;
import org.ftc9974.thorcore.util.MathUtilities;

public class Arm {

    private static final double HIGH_SHOULDER_LIMIT = 3.27,
                                MID_SHOULDER = 1.39,
                                LOW_SHOULDER_LIMIT = 0.493,
                                SAFE_TO_YAW = 0.6,
                                JAW0_OPEN_POS = MathUtilities.map(1910, 500, 2500, 0, 1),
                                JAW0_PUSH_POS = MathUtilities.map(938, 500, 2500, 0, 1),
                                JAW0_CLOSED_POS = MathUtilities.map(700, 500, 2500, 0, 1),
                                JAW1_OPEN_POS = MathUtilities.map(800, 500, 2500, 0, 1),
                                JAW1_READY_POS = MathUtilities.map(1790, 500, 2500, 0, 1),
                                JAW1_PUSH_POS = MathUtilities.map(1698, 500, 2500, 0, 1),
                                JAW1_CLOSED_POS = MathUtilities.map(1870, 500, 2500, 0, 1),
                                RAMP_RATE = 0.5; // seconds to full power

    @Hardware
    public DcMotorEx shoulder;

    @Hardware
    public ServoImplEx yaw, jaw0, jaw1;

    @Hardware
    public AnalogInput pot;

    private PIDF shoulderPid;
    private boolean closedLoopEnabled;

    private boolean grabberOpen;
    private boolean yawState;

    public double lastShoulderPower;
    private long lastShoulderUpdateTime;

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
        yaw.setPwmRange(new PwmControl.PwmRange(
                1085, // wide
                1955  // tall
        ));

        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //shoulder.setDirection(DcMotorSimple.Direction.REVERSE);

        shoulderPid = new PIDF(2, 0, 0, 0);
        shoulderPid.setNominalOutputForward(0.1);
        shoulderPid.setNominalOutputReverse(-0.1);
        shoulderPid.setPeakOutputForward(1);
        shoulderPid.setPeakOutputReverse(-1);
        shoulderPid.setAtTargetThreshold(0.05);
    }

    void grab() {
        grabberOpen = false;
        jaw0.setPosition(JAW0_CLOSED_POS);
        jaw1.setPosition(JAW1_CLOSED_POS);
    }

    void release() {
        grabberOpen = true;
        jaw0.setPosition(JAW0_OPEN_POS);
        if (yawState) {
            configureForPush();
        } else {
            if (getArmPosition() < MID_SHOULDER) {
                jaw1.setPosition(JAW1_OPEN_POS);
            } else {
                jaw1.setPosition(JAW1_READY_POS);
            }
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

    void configureForWide() {
        if (safeToYaw()) {
            yawState = true;
            yaw.setPosition(0);
        }
    }

    boolean safeToYaw() {
        return getArmPosition() < SAFE_TO_YAW;
    }

    void configureForTall() {
        yawState = false;
        yaw.setPosition(1);
    }

    void setShoulderPower(double power) {
        if ((power < 0 && getArmPosition() < MID_SHOULDER) || (power > 0 && getArmPosition() > MID_SHOULDER)) {
            power = MathUtilities.constrain(power, -0.5, 0.5);
        }
        double motorPower = power;

        long now = SystemClock.uptimeMillis();
        double deltaTime = (now - lastShoulderUpdateTime) / 1000.0;
        double input = lastShoulderPower - motorPower;
        double rampFactor = deltaTime / RAMP_RATE;
        if (Math.abs(input) < rampFactor) {
            rampFactor = input;
        }
        lastShoulderPower -= Math.copySign(rampFactor, input);
        lastShoulderPower = MathUtilities.constrain(lastShoulderPower, -1, 1);
        lastShoulderUpdateTime = now;

        if (getArmPosition() < LOW_SHOULDER_LIMIT) {
            lastShoulderPower = Math.max(0, lastShoulderPower);
        } else if (yawState && getArmPosition() > SAFE_TO_YAW) {
            lastShoulderPower = Math.min(0, lastShoulderPower);
        } else if (getArmPosition() > HIGH_SHOULDER_LIMIT) {
            lastShoulderPower = Math.min(0, lastShoulderPower);
        }

        shoulder.setPower(lastShoulderPower);
    }

    double getArmPosition() {
        return pot.getVoltage();
    }

    void setClosedLoopEnabled(boolean enabled) {
        closedLoopEnabled = enabled;
    }

    boolean isClosedLoopEnabled() {
        return closedLoopEnabled;
    }

    void setTargetPosition(double target) {
        shoulderPid.setSetpoint(target);
    }

    double getTargetPosition() {
        return shoulderPid.getSetpoint();
    }

    public double lastPIDError() {
        return shoulderPid.getLastError();
    }

    public boolean shoulderAtTarget() {
        return shoulderPid.atTarget();
    }

    public void update() {
        if (closedLoopEnabled) {
            setShoulderPower(shoulderPid.update(getArmPosition()));
        }
        if (grabberOpen && !yawState) {
            if (getArmPosition() < MID_SHOULDER) {
                jaw1.setPosition(JAW1_OPEN_POS);
            } else {
                jaw1.setPosition(JAW1_READY_POS);
            }
        }
    }
}
