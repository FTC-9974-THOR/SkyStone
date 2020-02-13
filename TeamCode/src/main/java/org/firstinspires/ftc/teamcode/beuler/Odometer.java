package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;

public class Odometer {

    @Hardware
    public ServoImplEx gearServo;

    private DcMotorEx encoder;

    public Odometer(HardwareMap hw, DcMotorEx encoder) {
        Realizer.realize(this, hw);
        this.encoder = encoder;

        gearServo.setPwmRange(new PwmControl.PwmRange(1875, 2160));
    }

    public void extend() {
        gearServo.setPosition(1);
    }

    public void retract() {
        gearServo.setPosition(0);
    }

    public void resetOdometer() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getOdometerPosition() {
        return 50.8 * Math.PI * (encoder.getCurrentPosition() / 8192d);
    }
}
