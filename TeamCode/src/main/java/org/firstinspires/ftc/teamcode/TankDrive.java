package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Tank Drive")
public class TankDrive extends OpMode {

    DcMotor left, right;

    @Override
    public void init() {
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");

        left.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        left.setPower(-gamepad1.left_stick_y);
        right.setPower(-gamepad1.right_stick_y);
    }
}
