package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;

public class StoneArm {

    @Hardware
    public ServoImplEx pivot, grip;

    public StoneArm(HardwareMap hw) {
        Realizer.realize(this, hw);
        pivot.setPwmRange(new PwmControl.PwmRange(1180, 1950));
        grip.setPwmRange(new PwmControl.PwmRange(1050, 1500));
    }

    public void retract() {
        pivot.setPosition(1);
    }

    public void lift() {
        pivot.setPosition(1);
    }

    public void place() {
        pivot.setPosition(0.3);
    }

    public void lower() {
        pivot.setPosition(0);
    }

    public void grab() {
        grip.setPosition(0);
    }

    public void slip() {
        grip.setPosition(0.3);
    }

    public void release() {
        grip.setPosition(1);
    }
}
