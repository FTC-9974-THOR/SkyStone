package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.control.navigation.OdometerNavSource;
import org.ftc9974.thorcore.control.navigation.PIDFMovementStrategy;
import org.ftc9974.thorcore.control.navigation.SynchronousNavigator;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;
import org.ftc9974.thorcore.util.MathUtilities;
import org.ftc9974.thorcore.util.TimingUtilities;

import java.util.List;

@Autonomous(name = "Red Foundation", group = "A")
public class RedOdometerFoundationAuto extends LinearOpMode {

    private MecanumDrive rb;
    private Intake intake;
    private OdometerNavSource odometerNavSource;
    private PIDFMovementStrategy pidfMovementStrategy;
    private SynchronousNavigator navigator;
    private ServoImplEx gearServo;
    private FoundationClaw foundationClaw;
    private ParkingTape parkingTape;

    private List<LynxModule> revHubs;

    private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();
        rb = new MecanumDrive(hardwareMap);
        rb.setAxisInversion(true, true, false);

        intake = new Intake(hardwareMap);

        odometerNavSource = new OdometerNavSource(
                hardwareMap,
                intake.intake0,
                intake.intake1,
                new Vector2(MathUtilities.inchesToMM(-9) + 25, MathUtilities.inchesToMM(-9) + 155),
                new Vector2(MathUtilities.inchesToMM(9) - 25, MathUtilities.inchesToMM(9) - 188),
                50,
                8192);
        odometerNavSource.setXInversion(false);
        odometerNavSource.setYInversion(true);
        pidfMovementStrategy = new PIDFMovementStrategy(
                0.003, 0, 0.00005, 0,
                0.0025, 0, 0.0001, 0,
                0.6 * 5 , 0, 0.6 / 8, 0,
                0.23,
                0.23,
                0.1,
                20,
                20,
                0.02
        );
        pidfMovementStrategy.setContinuity(-Math.PI, Math.PI);
        pidfMovementStrategy.setXPeriod(0.05);
        pidfMovementStrategy.setYPeriod(0.05);
        navigator = new SynchronousNavigator(odometerNavSource, rb, pidfMovementStrategy);
        navigator.setEnabled(false);

        gearServo = hardwareMap.get(ServoImplEx.class, "O-gearServo");
        gearServo.setPwmRange(new PwmControl.PwmRange(1875, 2160));
        gearServo.setPosition(1);

        foundationClaw = new FoundationClaw(hardwareMap);
        foundationClaw.retract();

        parkingTape = new ParkingTape(hardwareMap);

        revHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule revHub : revHubs) {
            revHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        timer = new ElapsedTime();

        odometerNavSource.resetOdometers();
        while (!isStopRequested() && !isStarted()) {
            telemetry.addLine("Ready.");
            clearHubCaches();
            telemetry.addData("Current Position", odometerNavSource.getLocation());
            telemetry.update();
        }
        odometerNavSource.resetOdometers();

        foundationClaw.ready();

        navigator.setTarget(new Vector2(-300, -780), 0);
        navigator.setEnabled(true);
        do {
            clearHubCaches();
            if (odometerNavSource.getLocation().getY() < -500) {
                pidfMovementStrategy.setSpeedLimit(0.25);
            }
            telemetry.addData("Current Position", odometerNavSource.getLocation());
            telemetry.addData("Target Position", navigator.getTargetPosition());
            telemetry.addData("Current Heading", odometerNavSource.getHeading());
            telemetry.addData("Target Heading", navigator.getTargetHeading());
            telemetry.update();
            navigator.update();
        } while (!isStopRequested() && !navigator.atTarget());
        if (isStopRequested()) return;
        pidfMovementStrategy.setSpeedLimit(1);

        foundationClaw.extend();
        TimingUtilities.sleep(this, 0.75, null, null);
        if (isStopRequested()) return;

        timer.reset();
        navigator.setAllowMovement(false);
        navigator.setTargetHeading(Math.toRadians(-10));
        do {
            clearHubCaches();
            navigator.update();
        } while (!isStopRequested() && !pidfMovementStrategy.atHeadingTarget() && timer.seconds() < 1.8);
        if (isStopRequested()) return;

        odometerNavSource.resetOdometers();
        navigator.setAllowMovement(true);
        navigator.setTargetPosition(new Vector2(125, 600));
        do {
            clearHubCaches();
            navigator.update();
        } while (!isStopRequested() && !navigator.atTarget());
        if (isStopRequested()) return;

        timer.reset();
        navigator.setAllowMovement(false);
        navigator.setTargetHeading(-0.5 * Math.PI);
        do {
            clearHubCaches();
            navigator.update();
        } while (!isStopRequested() && !pidfMovementStrategy.atHeadingTarget() && timer.seconds() < 4);
        if (isStopRequested()) return;
        odometerNavSource.resetOdometers();

        timer.reset();
        navigator.setAllowMovement(true);
        navigator.setTargetPosition(new Vector2(0, -500));
        do {
            clearHubCaches();
            navigator.update();
        } while (!isStopRequested() &&
                (timer.seconds() < 0.15 || Math.abs(intake.intake1.getVelocity()) / 8192 > 0.5)
                && timer.seconds() < 3);
        if (isStopRequested()) return;

        parkingTape.setPower(1);
        timer.reset();
        foundationClaw.retract();
        TimingUtilities.sleep(this, 0.5, null, null);
        if (isStopRequested()) return;

        navigator.setTargetPosition(new Vector2(400, 50));
        do {
            clearHubCaches();
            navigator.update();
        } while (!isStopRequested() && !navigator.atTarget());
        if (isStopRequested()) return;

        navigator.setAllowMovement(false);
        navigator.setTargetHeading(-0.5 * Math.PI - Math.toRadians(34));
        do {
            clearHubCaches();
            navigator.update();
        } while (!isStopRequested() && !pidfMovementStrategy.atHeadingTarget() && timer.seconds() < 2);
        rb.drive(0, 0, 0);

        TimingUtilities.blockUntil(this, () -> timer.seconds() > 5, null, null);
        parkingTape.setPower(0);
    }

    private void clearHubCaches() {
        for (LynxModule revHub : revHubs) {
            revHub.clearBulkCache();
        }
    }
}
