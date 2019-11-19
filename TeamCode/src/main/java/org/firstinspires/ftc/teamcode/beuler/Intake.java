package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.robot.Motor;

public class Intake {

    @Hardware
    public DcMotorEx intake0, intake1;

    public Intake(HardwareMap hw) {
        Realizer.realize(this, hw);
    }

    public void intake(double speed) {
        intake0.setPower(speed);
        intake1.setPower(speed);
    }

    public void outtake(double speed) {
        intake0.setPower(-speed);
        intake1.setPower(-speed);
    }

    public void stop() {
        intake0.setPower(0);
        intake1.setPower(0);
    }
}
