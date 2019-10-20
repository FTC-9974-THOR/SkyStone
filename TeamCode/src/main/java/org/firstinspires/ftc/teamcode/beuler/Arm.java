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
import org.ftc9974.thorcore.util.UpdateLoopHandler;

public class Arm {

    private static final double HIGH_SHOULDER_LIMIT = 2.5,
                                MID_SHOULDER = 1.36,
                                LOW_SHOULDER_LIMIT = 0.3;

    @Hardware
    public Motor shoulder;

    @Hardware
    public ServoImplEx wrist, yaw, jaw0, jaw1;

    @Hardware
    public AnalogInput pot;

    private PIDF shoulderPid;
    private UpdateLoopHandler updateLoopHandler;

    public Arm(HardwareMap hw) {
        Realizer.realize(this, hw);
        jaw0.setPwmRange(new PwmControl.PwmRange(800, 1500));
        jaw1.setPwmRange(new PwmControl.PwmRange(1450, 2200));
        yaw.setPwmRange(new PwmControl.PwmRange(1050, 1860));

        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shoulderPid = new PIDF(1.5, 0, 0, 0);
        shoulderPid.setNominalOutputForward(0.1);
        shoulderPid.setNominalOutputReverse(-0.1);
        shoulderPid.setPeakOutputForward(0.5);
        shoulderPid.setPeakOutputReverse(-0.5);
        shoulderPid.setAtTargetThreshold(0.05);

        shoulderPid.setInputFunction(this::getArmPosition);
        shoulderPid.setOutputFunction(this::setShoulderPower);

        updateLoopHandler = new UpdateLoopHandler(shoulderPid::update);
        updateLoopHandler.startOnOpModeStart();
    }

    void grab() {
        jaw0.setPosition(1);
        jaw1.setPosition(0);
    }

    void release() {
        jaw0.setPosition(0);
        jaw1.setPosition(1);
    }

    void configureForWide() {
        yaw.setPosition(1);
    }

    void configureForTall() {
        yaw.setPosition(0);
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
            shoulder.setPower(power);
        }
    }

    double getArmPosition() {
        return pot.getVoltage();
    }

    void setClosedLoopEnabled(boolean enabled) {
        updateLoopHandler.setEnabled(enabled);
    }

    boolean isClosedLoopEnabled() {
        return updateLoopHandler.isEnabled();
    }

    void setTargetPosition(double target) {
        shoulderPid.setSetpoint(target);
    }

    double getTargetPosition() {
        return shoulderPid.getSetpoint();
    }
}
