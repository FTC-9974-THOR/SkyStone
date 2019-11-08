package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.robot.Motor;

public class FoundationClaw {

    private static final int EXTENDED_POSITION = -120;

    @Hardware
    public Motor claw;

    private PIDF pid;
    private boolean closedLoopEnabled;

    public FoundationClaw(HardwareMap hw) {
        Realizer.realize(this, hw);
        claw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        claw.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pid = new PIDF(0.01, 0, 0, 0);
        pid.setAtTargetThreshold(5);
        pid.setNominalOutputForward(0.1);
        pid.setNominalOutputReverse(-0.1);
        pid.setPeakOutputForward(0.3);
        pid.setPeakOutputReverse(-0.3);

        pid.setSetpoint(0);
        pid.setPeriod(0.1);
        pid.setIfPeriodAppliesOnlyToDTerm(true);
        closedLoopEnabled = true;
        claw.setStallThreshold(0.0001);
    }

    // I could make these functions automatically switch the PIDF on and off,
    // but I prefer to make the functions do only what they say they do, and
    // nothing more. It helps prevent confusion.

    public void extend() {
        pid.setSetpoint(EXTENDED_POSITION);
    }

    public void retract() {
        pid.setSetpoint(-40);
    }

    public void setTarget(int target) {
        pid.setSetpoint(target);
    }

    public boolean atTarget() {
        return pid.atTarget();
    }

    public int getCurrentPosition() {
        return claw.getCurrentPosition();
    }

    public void setClosedLoopEnabled(boolean enabled) {
        closedLoopEnabled = enabled;
    }

    public boolean isClosedLoopEnabled() {
        return closedLoopEnabled;
    }

    public void setPower(double power) {
        claw.setPower(power);
    }

    public boolean isStalled() {
        return claw.isStalled();
    }

    public void homeEncoder() {
        claw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        claw.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {
        if (closedLoopEnabled) {
            claw.setPower(pid.update(getCurrentPosition()));
        }
    }
}
