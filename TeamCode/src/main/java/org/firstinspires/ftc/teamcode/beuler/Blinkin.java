package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Blinkin {

    private RevBlinkinLedDriver blinkin;

    public Blinkin(HardwareMap hw) {
        blinkin = hw.get(RevBlinkinLedDriver.class, "blinkin");
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        blinkin.setPattern(pattern);
    }

    public void defaultPattern() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE);
    }

    public void lamp() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
    }
}
