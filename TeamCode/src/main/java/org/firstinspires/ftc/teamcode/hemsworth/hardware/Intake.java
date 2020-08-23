package org.firstinspires.ftc.teamcode.hemsworth.hardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.util.MathUtilities;

public class Intake {

    @Hardware
    public ServoImplEx motorController;

    @Hardware
    public DigitalChannel loadSensor;

    public Intake(HardwareMap hw) {
        Realizer.realize(this, hw);
        motorController.setPwmRange(new PwmControl.PwmRange(
                1000,
                2000
        ));

        loadSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public void setPower(double power) {
        if (isStonePresent()) {
            power = Math.min(0, power);
        }
        motorController.setPosition(MathUtilities.map(power, -1, 1, 1, 0));

    }

    public boolean isStonePresent() {
        return !loadSensor.getState();
    }
}
