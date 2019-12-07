package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.ftc9974.thorcore.control.navigation.IMUNavSource;
import org.ftc9974.thorcore.robot.drivetrains.KiwiDrive;

//@Disabled
@TeleOp(name = "Thorbot")
public class ThorbotOpMode extends OpMode {

    private IMUNavSource navSource;
    private KiwiDrive rb;

    @Override
    public void init() {
        navSource = new IMUNavSource(hardwareMap);
        rb = new KiwiDrive(hardwareMap, new PIDFCoefficients(0.1, 0, 0, 0), navSource, (byte) 0b000);
        rb.setFieldRelative(false);
        rb .setTurningPidfActive(true);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Calibration", navSource.getCalibrationStatus().toString());
    }

    @Override
    public void loop() {
        rb.drive(-gamepad1.right_stick_x, gamepad1.right_stick_y, -gamepad1.left_stick_x);
    }
}
