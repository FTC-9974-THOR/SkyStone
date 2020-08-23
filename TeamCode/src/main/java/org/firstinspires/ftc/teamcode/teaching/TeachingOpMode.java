package org.firstinspires.ftc.teamcode.teaching;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.control.navigation.IMUNavSource;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;
import org.ftc9974.thorcore.robot.drivetrains.TankDrive2Wheel;

public class TeachingOpMode extends OpMode {

    MecanumDrive rb;
    IMUNavSource navSource;
    PIDF controller;

    @Override
    public void init() {
        rb = new MecanumDrive(hardwareMap);
        navSource = new IMUNavSource(hardwareMap);
        controller = new PIDF(1 / Math.toRadians(20), 0, 1, 0);
        controller.setContinuityRange(0, 2 * Math.PI);
        controller.setContinuous(true);
        controller.setPeakOutputForward(1);
        controller.setPeakOutputReverse(-1);
        controller.setNominalOutputForward(0.1);
        controller.setNominalOutputReverse(-0.1);
    }

    @Override
    public void start() {
        controller.setSetpoint(Math.PI);
    }

    @Override
    public void loop() {
        rb.drive(0, 0, controller.update(navSource.getHeading()));
    }
}
