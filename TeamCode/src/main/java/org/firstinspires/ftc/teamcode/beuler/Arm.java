package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.robot.Motor;
import org.ftc9974.thorcore.util.MathUtilities;

public class Arm {

    private static final double HIGH_SHOULDER_LIMIT = 3.3,
                                MID_SHOULDER = 1.39,
                                LOW_SHOULDER_LIMIT = 0.493,
                                SAFE_TO_YAW = 0.6;

    @Hardware
    public Motor shoulder;

    @Hardware
    public ServoImplEx yaw, jaw0, jaw1;

    @Hardware
    public AnalogInput pot;

    private PIDF shoulderPid;
    private boolean closedLoopEnabled;

    public Arm(HardwareMap hw) {
        Realizer.realize(this, hw);
        jaw0.setPwmRange(new PwmControl.PwmRange(
                1145, // closed
                2200 // open
        ));
        jaw1.setPwmRange(new PwmControl.PwmRange(
                1935, // open
                2020  // closed
        ));
        yaw.setPwmRange(new PwmControl.PwmRange(
                1100, // wide
                1950  // tall
        ));

        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shoulderPid = new PIDF(1.5, 0, 0, 0);
        shoulderPid.setNominalOutputForward(0.1);
        shoulderPid.setNominalOutputReverse(-0.1);
        shoulderPid.setPeakOutputForward(0.5);
        shoulderPid.setPeakOutputReverse(-0.5);
        shoulderPid.setAtTargetThreshold(0);

        shoulderPid.setInputFunction(this::getArmPosition);
        shoulderPid.setOutputFunction(this::setShoulderPower);
    }

    void grab() {
        jaw0.setPosition(0);
        jaw1.setPosition(1);
    }

    void release() {
        jaw0.setPosition(1);
        jaw1.setPosition(0);
    }

    void configureForWide() {
        if (getArmPosition() < SAFE_TO_YAW) {
            yaw.setPosition(0);
        }
    }

    void configureForTall() {
        yaw.setPosition(1);
    }

    void setShoulderPower(double power) {
        if (getArmPosition() < LOW_SHOULDER_LIMIT) {
            shoulder.setPower(Math.max(0, power));
        } else if (getArmPosition() > HIGH_SHOULDER_LIMIT) {
            shoulder.setPower(Math.min(0, power));
        } else {
            if ((power > 0 && getArmPosition() < MID_SHOULDER) || (power < 0 && getArmPosition() > MID_SHOULDER)) {
                power *= 1.3;
            }
            if (power > 0 && getArmPosition() > MID_SHOULDER) {
                power = Math.min(power, 1);
            }
            shoulder.setPower(power);
        }
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

    public void update() {
        if (closedLoopEnabled) {
            shoulderPid.update();
        }
        if (getArmPosition() > SAFE_TO_YAW) {
            configureForTall();
        }
    }
}
