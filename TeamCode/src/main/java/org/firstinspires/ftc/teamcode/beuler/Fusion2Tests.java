package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Fusion2;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.control.navigation.VuMarkNavSource;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;

@Autonomous(name = "Fusion 2 Tests", group = "RND")
@Disabled
public class Fusion2Tests extends LinearOpMode {

    private MecanumDrive rb;
    private Fusion2 fusion2;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();
        rb = new MecanumDrive(hardwareMap);
        rb.setAxisInversion(true, true, false);
        rb.setEncoderInversion(true, true, false, false);
        rb.resetEncoders();

        fusion2 = new Fusion2(hardwareMap, rb);

        if (fusion2.getNavSource().isFallback()) {
            telemetry.log().add("Warning: Primary IMU failure, using fallback!");
            telemetry.update();
        }

        rb.resetEncoders();

        while (!isStarted() && !isStopRequested()) {
            int[] encoderPositions = rb.getEncoderPositions();
            telemetry.addData("FL", encoderPositions[0]);
            telemetry.addData("FR", encoderPositions[1]);
            telemetry.addData("BL", encoderPositions[2]);
            telemetry.addData("BR", encoderPositions[3]);
            telemetry.update();
        }

        fusion2.drive(this, new Vector2(1000, 500), () -> {
            int[] encoderPositions = rb.getEncoderPositions();
            telemetry.addData("FL", encoderPositions[0]);
            telemetry.addData("FR", encoderPositions[1]);
            telemetry.addData("BL", encoderPositions[2]);
            telemetry.addData("BR", encoderPositions[3]);
            telemetry.update();
        }, 1);
    }
}
