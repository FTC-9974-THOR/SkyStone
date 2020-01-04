package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Swerve Test")
public class SwerveTestOpMode extends OpMode {

    private DcMotorEx drive, slew;

    @Override
    public void init() {
        drive = hardwareMap.get(DcMotorEx.class, "drive");
        slew = hardwareMap.get(DcMotorEx.class, "slew");
    }

    @Override
    public void loop() {
        drive.setPower(gamepad1.right_stick_y);
        slew.setPower(gamepad1.left_stick_x);
    }
}
