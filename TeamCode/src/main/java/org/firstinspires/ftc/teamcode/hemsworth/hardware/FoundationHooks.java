package org.firstinspires.ftc.teamcode.hemsworth.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;

public class FoundationHooks {

    @Hardware
    public ServoImplEx leftHook, rightHook;

    public FoundationHooks(HardwareMap hw) {
        Realizer.realize(this, hw);
        leftHook.setPwmRange(new PwmControl.PwmRange(
                1276,
                1955
        ));
        rightHook.setPwmRange(new PwmControl.PwmRange(
                912,
                1523
        ));
    }

    public void extend() {
        leftHook.setPosition(1);
        rightHook.setPosition(0);
    }

    public void retract() {
        leftHook.setPosition(0);
        rightHook.setPosition(1);
    }
}
