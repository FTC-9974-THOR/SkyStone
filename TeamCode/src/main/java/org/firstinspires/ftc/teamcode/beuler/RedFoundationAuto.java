package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Fusion2;
import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.control.navigation.IMUNavSource;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;
import org.ftc9974.thorcore.util.TimingUtilities;

@Autonomous(name = "Red Foundation", group = "A")
public class RedFoundationAuto extends LinearOpMode {

    private MecanumDrive rb;
    private Fusion2 fusion2;
    private FoundationClaw foundationClaw;
    private ParkingTape parkingTape;

    private PIDF headingPid;

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

        foundationClaw = new FoundationClaw(hardwareMap);

        parkingTape = new ParkingTape(hardwareMap);

        headingPid = fusion2.headingPid;
        //headingPid.setNominalOutputForward(0.15);
        //headingPid.setNominalOutputReverse(-0.15);
        headingPid.setAtTargetThreshold(Math.toRadians(0.5));

        while (!isStopRequested() && !isStarted()) {
            telemetry.addLine("Ready.");
            telemetry.update();
        }

        foundationClaw.ready();

        fusion2.drive(this, new Vector2(0, -350), null, 1);
        if (isStopRequested()) return;

        fusion2.drive(this, new Vector2(-400, 0), null, 1);
        if (isStopRequested()) return;

        fusion2.drive(this, new Vector2(0, -430), null, 0.5);
        if (isStopRequested()) return;

        foundationClaw.extend();

        TimingUtilities.sleep(this, 0.5, null, null);
        if (isStopRequested()) return;

        fusion2.drive(this, new Vector2(100, 600), null, 1);
        if (isStopRequested()) return;

        {
            headingPid.setSetpoint(-0.5 * Math.PI);
            headingPid.setAtTargetThreshold(Math.toRadians(10));
            while (!isStopRequested()) {
                foundationClaw.update();
                // idea: stop driving forwards after turning a certain amount?
                rb.drive(0, 0.4, headingPid.update());
                if (headingPid.atTarget()) {
                    break;
                }
            }
            headingPid.setAtTargetThreshold(Math.toRadians(0.5));
        }
        rb.drive(0, 0, 0);
        if (isStopRequested()) return;

        foundationClaw.retract();

        fusion2.drive(this, new Vector2(400, 0), null, 1);
        if (isStopRequested()) return;

        fusion2.drive(this, new Vector2(0, -280), null, 0.4);
        if (isStopRequested()) return;

        fusion2.drive(this, new Vector2(0, 50), null, 1);
        if (isStopRequested()) return;

        fusion2.drive(this, new Vector2(-300, 0), null, 0.5);
        if (isStopRequested()) return;

        parkingTape.setPower(1);

        TimingUtilities.runAfterDelay(() -> {
            parkingTape.setPower(0);
        }, 5000);

        rb.drive(-0.2, 0, 0);
        TimingUtilities.sleep(this, 2, null, null);
        rb.drive(0, 0, 0);

        TimingUtilities.blockUntil(this, this::isStopRequested, null, null);
    }
}
