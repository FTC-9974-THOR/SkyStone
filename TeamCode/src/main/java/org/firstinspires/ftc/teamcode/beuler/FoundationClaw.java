package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.robot.Motor;
import org.ftc9974.thorcore.robot.StallDetector;

public class FoundationClaw {

    @Hardware
    public ServoImplEx claw0;
    @Hardware
    public ServoImplEx claw1;

    public FoundationClaw(HardwareMap hw) {
        Realizer.realize(this, hw);
        claw0.setPwmRange(new PwmControl.PwmRange(1100, 2020));
        claw1.setPwmRange(new PwmControl.PwmRange(1320, 1900));
    }

    public void extend() {
        claw0.setPosition(1);
        claw1.setPosition(0);
    }

    public void retract() {
        claw0.setPosition(0);
        claw1.setPosition(1);
    }

    public void ready() {
        claw0.setPosition(0.3);
        claw1.setPosition(0.7);
    }

    @Deprecated
    public void setTarget(int target) {

    }

    @Deprecated
    public boolean atTarget() {
        return true;
    }

    @Deprecated
    public int getCurrentPosition() {
        return 0;
    }

    @Deprecated
    public void setClosedLoopEnabled(boolean enabled) {

    }

    @Deprecated
    public boolean isClosedLoopEnabled() {
        return true;
    }

    @Deprecated
    public void setPower(double power) {

    }

    @Deprecated
    public boolean isStalled() {
        return true;
    }

    @Deprecated
    public void homeEncoder() {

    }

    public void update() {
    }
}
