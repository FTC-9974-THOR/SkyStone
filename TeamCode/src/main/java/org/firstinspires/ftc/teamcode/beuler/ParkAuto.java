package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;
import org.ftc9974.thorcore.util.TimingUtilities;

@Autonomous(name = "Park")
public class ParkAuto extends LinearOpMode {

    private enum Side {
        LEFT,
        RIGHT
    }

    private MecanumDrive rb;
    private AutonomousSensorManager asm;

    @Override
    public void runOpMode() throws InterruptedException {
        rb = new MecanumDrive(hardwareMap);
        rb = new MecanumDrive(hardwareMap);
        rb.setAxisInversion(true, true, false);

        asm = new AutonomousSensorManager(hardwareMap);

        Side side = (asm.getLeftDistance() < asm.getRightDistance()) ? Side.LEFT : Side.RIGHT;

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Side", side);
            telemetry.update();
        }

        if (side == Side.LEFT) {
            rb.drive(0.5, 0, 0);
        } else {
            rb.drive(-0.5, 0, 0);
        }

        TimingUtilities.sleep(this, 0.5, null, null);
        rb.drive(0, 0, 0);
        if (isStopRequested()) return;

        rb.drive(0, 0.3, 0);
        TimingUtilities.blockUntil(this, asm::onTape, null, null);
        rb.drive(0, 0, 0);
    }
}
