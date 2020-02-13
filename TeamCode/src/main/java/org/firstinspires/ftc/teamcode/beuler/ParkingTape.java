package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;

public class ParkingTape {

    @Hardware
    public CRServo extension0, extension1;

    public ParkingTape(HardwareMap hw) {
        Realizer.realize(this, hw);
        extension1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power) {
        extension0.setPower(power);
        extension1.setPower(power);
    }
}
