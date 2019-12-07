package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;

@TeleOp(name = "Practice Mecanum", group = "Practice")
public class PracticeMecanumTeleOp extends OpMode {

    private MecanumDrive rb;

    @Hardware
    public DcMotorEx intake0, intake1;

    @Override
    public void init() {
        Realizer.realize(this, hardwareMap);
        rb = new MecanumDrive(hardwareMap);
    }

    @Override
    public void loop() {
        rb.drive(-gamepad1.right_stick_x, -gamepad1.right_stick_y, -gamepad1.left_stick_x);

        if (gamepad1.left_trigger > 0.8) {
            intake0.setPower(-1);
            intake1.setPower(1);
        } else if (gamepad1.right_trigger > 0.8) {
            intake0.setPower(1);
            intake1.setPower(-1);
        } else {
            intake0.setPower(0);
            intake1.setPower(0);
        }
    }
}
