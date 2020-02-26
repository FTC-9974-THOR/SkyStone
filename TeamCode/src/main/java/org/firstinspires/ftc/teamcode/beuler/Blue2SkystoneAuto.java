package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.StonePosition;
import org.firstinspires.ftc.teamcode.StoneVision;
import org.firstinspires.ftc.teamcode.VuforiaKey;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.control.navigation.OdometerNavSource;
import org.ftc9974.thorcore.control.navigation.PIDFMovementStrategy;
import org.ftc9974.thorcore.control.navigation.SynchronousNavigator;
import org.ftc9974.thorcore.control.navigation.VuMarkNavSource;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;
import org.ftc9974.thorcore.util.MathUtilities;
import org.ftc9974.thorcore.util.TimingUtilities;

import java.util.List;

@Autonomous(name = "Blue 2 Skystone Auto", group = "RND")
public class Blue2SkystoneAuto extends LinearOpMode {

    private MecanumDrive rb;
    private Intake intake;
    private OdometerNavSource odometerNavSource;
    private PIDFMovementStrategy pidfMovementStrategy;
    private SynchronousNavigator navigator;

    private VuMarkNavSource vuforia;
    private StoneVision stoneVision;

    private StoneArm stoneArm;

    private ServoImplEx gearServo;

    private List<LynxModule> revHubs;

    private Arm arm;

    private FoundationClaw foundationClaw;

    private ParkingTape parkingTape;

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
                0.18,
                0.18,
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

        OpenGLMatrix phoneLocation = OpenGLMatrix
                .translation(0, 0, 0)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES, 90, 0, 0));

        vuforia = new VuMarkNavSource(hardwareMap.appContext, VuforiaKey.KEY, VuforiaLocalizer.CameraDirection.BACK, phoneLocation);

        stoneVision = new StoneVision(vuforia.getVuforiaLocalizer(), StoneVision.Tunings.BLUE);

        stoneArm = new StoneArm(hardwareMap);

        gearServo = hardwareMap.get(ServoImplEx.class, "O-gearServo");
        gearServo.setPwmRange(new PwmControl.PwmRange(1875, 2160));
        gearServo.setPosition(1);

        stoneArm.release();
        stoneArm.retract();

        revHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule revHub : revHubs) {
            revHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        arm = new Arm(hardwareMap);
        arm.release();

        foundationClaw = new FoundationClaw(hardwareMap);
        foundationClaw.retract();

        parkingTape = new ParkingTape(hardwareMap);

        pidfMovementStrategy.setSpeedLimit(1);

        odometerNavSource.resetOdometers();

        while (!isStopRequested() && !isStarted()) {
            telemetry.addLine("Ready.");
            telemetry.update();
        }
        if (isStopRequested()) return;

        odometerNavSource.resetOdometers();

        stoneVision.beginProcessing();

        navigator.setTarget(new Vector2(700, 0), 0);
        navigator.setEnabled(true);
        pidfMovementStrategy.setSpeedLimit(0.55);

        while (!isStopRequested()) {
            clearHubCaches();
            telemetry.addData("Current Position", odometerNavSource.getLocation());
            telemetry.addData("Target Position", navigator.getTargetPosition());
            telemetry.addData("Current Heading", odometerNavSource.getHeading());
            telemetry.addData("Target Heading", navigator.getTargetHeading());
            telemetry.update();
            navigator.update();
            if (stoneVision.isProcessingComplete()) {
                telemetry.log().add("Processing completed in %.3fs", stoneVision.time);
                telemetry.log().add("Stone Position: %s", stoneVision.getStonePosition().toString());
                telemetry.update();
                switch (stoneVision.getStonePosition()) {
                    case LEFT:
                        navigator.setTargetPosition(new Vector2(710, 100));
                        break;
                    case CENTER:
                        navigator.setTargetPosition(new Vector2(710, -90));
                        break;
                    case RIGHT:
                        navigator.setTargetPosition(new Vector2(710, -280));
                        break;
                }
                break;
            }
        }
        if (isStopRequested()) return;

        do {
            clearHubCaches();
            telemetry.addData("Current Position", odometerNavSource.getLocation());
            telemetry.addData("Target Position", navigator.getTargetPosition());
            telemetry.addData("Current Heading", odometerNavSource.getHeading());
            telemetry.addData("Target Heading", navigator.getTargetHeading());
            telemetry.update();
            navigator.update();
        } while (!isStopRequested() && !navigator.atTarget());
        if (isStopRequested()) return;
        pidfMovementStrategy.setSpeedLimit(1);

        stoneArm.lower();
        TimingUtilities.sleep(this, 0.4, null, null);
        if (isStopRequested()) return;
        stoneArm.grab();
        TimingUtilities.sleep(this, 0.4, null, null);
        if (isStopRequested()) return;
        stoneArm.retract();

        // move under the bridge
        navigator.setStopAtEnd(false);
        navigator.setTargetPosition(new Vector2(550, 200));
        do {
            clearHubCaches();
            telemetry.addData("Current Position", odometerNavSource.getLocation());
            telemetry.addData("Target Position", navigator.getTargetPosition());
            telemetry.addData("Current Heading", odometerNavSource.getHeading());
            telemetry.addData("Target Heading", navigator.getTargetHeading());
            telemetry.update();
            navigator.update();
        } while (!isStopRequested() && !navigatorWithin(100));
        if (isStopRequested()) return;
        telemetry.log().add("Point 1");
        telemetry.update();

        navigator.setTargetPosition(new Vector2(550, 1400));
        do {
            clearHubCaches();
            telemetry.addData("Current Position", odometerNavSource.getLocation());
            telemetry.addData("Target Position", navigator.getTargetPosition());
            telemetry.addData("Current Heading", odometerNavSource.getHeading());
            telemetry.addData("Target Heading", navigator.getTargetHeading());
            telemetry.update();
            navigator.update();
        } while (!isStopRequested() && !navigatorWithin(100));
        if (isStopRequested()) return;
        telemetry.log().add("Point 2");
        telemetry.update();

        /*navigator.setTargetPosition(new Vector2(550, 1560));
        do {
            clearHubCaches();
            telemetry.addData("Current Position", odometerNavSource.getLocation());
            telemetry.addData("Target Position", navigator.getTargetPosition());
            telemetry.addData("Current Heading", odometerNavSource.getHeading());
            telemetry.addData("Target Heading", navigator.getTargetHeading());
            telemetry.update();
            navigator.update();
        } while (!isStopRequested() && !navigatorWithin(100));
        telemetry.log().add("Point 3");
        telemetry.update();*/

        navigator.setStopAtEnd(true);
        navigator.setTargetPosition(new Vector2(747, 1440 + 278));
        do {
            clearHubCaches();
            telemetry.addData("Current Position", odometerNavSource.getLocation());
            telemetry.addData("Target Position", navigator.getTargetPosition());
            telemetry.addData("Current Heading", odometerNavSource.getHeading());
            telemetry.addData("Target Heading", navigator.getTargetHeading());
            telemetry.update();
            navigator.update();
        } while (!isStopRequested() && !navigator.atTarget());
        if (isStopRequested()) return;
        telemetry.log().add("Point 4");
        telemetry.update();

        stoneArm.place();
        TimingUtilities.sleep(this, 0.5, null, null);
        stoneArm.release();
        TimingUtilities.sleep(this, 0.1, null, null);
        stoneArm.retract();
        TimingUtilities.sleep(this, 0.5, null, null);
        if (isStopRequested()) return;

        navigator.setStopAtEnd(false);
        navigator.setTargetPosition(new Vector2(600, 1400));
        do {
            clearHubCaches();
            telemetry.addData("Current Position", odometerNavSource.getLocation());
            telemetry.addData("Target Position", navigator.getTargetPosition());
            telemetry.addData("Current Heading", odometerNavSource.getHeading());
            telemetry.addData("Target Heading", navigator.getTargetHeading());
            telemetry.update();
            navigator.update();
        } while (!isStopRequested() && !navigatorWithin(100));
        if (isStopRequested()) return;
        telemetry.log().add("Point 1");
        telemetry.update();

        navigator.setStopAtEnd(true);
        switch (stoneVision.getStonePosition()) {
            case LEFT:
                navigator.setTargetPosition(new Vector2(600, 3 - 230 + 360));
                break;
            case CENTER:
                navigator.setTargetPosition(new Vector2(600, -225 - 120 + 230));
                break;
            case RIGHT:
                navigator.setTargetPosition(new Vector2(600, -425 - 150 + 260));
                break;
        }
        do {
            clearHubCaches();
            telemetry.addData("Current Position", odometerNavSource.getLocation());
            telemetry.addData("Target Position", navigator.getTargetPosition());
            telemetry.addData("Current Heading", odometerNavSource.getHeading());
            telemetry.addData("Target Heading", navigator.getTargetHeading());
            telemetry.update();
            if (odometerNavSource.getLocation().getY() > -200) {
                pidfMovementStrategy.setSpeedLimit(0.5);
            }
            navigator.update();
        } while (!isStopRequested() && !navigator.atTarget());
        if (isStopRequested()) return;
        pidfMovementStrategy.setSpeedLimit(1);
        telemetry.log().add("Point 2");
        telemetry.update();

        navigator.setAllowMovement(false);
        navigator.setTargetHeading(Math.PI);
        do {
            clearHubCaches();
            telemetry.addData("Current Position", odometerNavSource.getLocation());
            telemetry.addData("Target Position", navigator.getTargetPosition());
            telemetry.addData("Current Heading", odometerNavSource.getHeading());
            telemetry.addData("Target Heading", navigator.getTargetHeading());
            telemetry.update();
            navigator.update();
        } while (!isStopRequested() && !pidfMovementStrategy.atHeadingTarget());
        if (isStopRequested()) return;

        odometerNavSource.resetOdometers();

        arm.setShoulderPower(-0.4);
        TimingUtilities.runAfterDelay(() -> arm.setShoulderPower(0), 500);

        // strafe before intake
        navigator.setTargetPosition(new Vector2(-390, 0));
        navigator.setAllowMovement(true);
        do {
            clearHubCaches();
            telemetry.addData("Current Position", odometerNavSource.getLocation());
            telemetry.addData("Target Position", navigator.getTargetPosition());
            telemetry.addData("Current Heading", odometerNavSource.getHeading());
            telemetry.addData("Target Heading", navigator.getTargetHeading());
            telemetry.update();
            navigator.update();
        } while (!isStopRequested() && !navigator.atTarget());
        if (isStopRequested()) return;

        intake.intake(1);
        TimingUtilities.runAfterDelay(intake::stop, 1200);

        pidfMovementStrategy.setSpeedLimit(0.8);
        navigator.setTargetPosition(new Vector2(-390, 150));
        do {
            clearHubCaches();
            telemetry.addData("Current Position", odometerNavSource.getLocation());
            telemetry.addData("Target Position", navigator.getTargetPosition());
            telemetry.addData("Current Heading", odometerNavSource.getHeading());
            telemetry.addData("Target Heading", navigator.getTargetHeading());
            telemetry.update();
            navigator.update();
        } while (!isStopRequested() && !navigator.atTarget());
        if (isStopRequested()) return;
        pidfMovementStrategy.setSpeedLimit(1);

        arm.setShoulderPower(0.4);
        TimingUtilities.runAfterDelay(() -> arm.setShoulderPower(0), 750);
        navigator.setTargetPosition(new Vector2(25, -300));
        do {
            clearHubCaches();
            if (arm.stowExtent.isPressed()) {
                arm.setShoulderPower(0);
                arm.grab();
            }
            telemetry.addData("Current Position", odometerNavSource.getLocation());
            telemetry.addData("Target Position", navigator.getTargetPosition());
            telemetry.addData("Current Heading", odometerNavSource.getHeading());
            telemetry.addData("Target Heading", navigator.getTargetHeading());
            telemetry.update();
            navigator.update();
        } while (!isStopRequested() && !navigator.atTarget());
        if (isStopRequested()) return;

        switch (stoneVision.getStonePosition()) {
            case LEFT:
                navigator.setTargetPosition(new Vector2(-68, -2000));
                break;
            case CENTER:
                navigator.setTargetPosition(new Vector2(-68, -2200));
                break;
            case RIGHT:
                navigator.setTargetPosition(new Vector2(-68, -2500));
                break;
        }
        do {
            clearHubCaches();
            telemetry.addData("Current Position", odometerNavSource.getLocation());
            telemetry.addData("Target Position", navigator.getTargetPosition());
            telemetry.addData("Current Heading", odometerNavSource.getHeading());
            telemetry.addData("Target Heading", navigator.getTargetHeading());
            telemetry.update();
            navigator.update();
        } while (!isStopRequested() && !navigator.atTarget());
        if (isStopRequested()) return;

        navigator.setAllowMovement(false);
        navigator.setTargetHeading(0.5 * Math.PI);
        do {
            clearHubCaches();
            telemetry.addData("Current Position", odometerNavSource.getLocation());
            telemetry.addData("Target Position", navigator.getTargetPosition());
            telemetry.addData("Current Heading", odometerNavSource.getHeading());
            telemetry.addData("Target Heading", navigator.getTargetHeading());
            telemetry.update();
            navigator.update();
        } while (!isStopRequested() && !pidfMovementStrategy.atHeadingTarget());
        if (isStopRequested()) return;

        odometerNavSource.resetOdometers();
        navigator.setAllowMovement(true);

        // account for turning drift
        if (stoneVision.getStonePosition() == StonePosition.RIGHT) {
            navigator.setTargetPosition(new Vector2(0, -300));
        } else {
            navigator.setTargetPosition(new Vector2(0, -250));
        }
        foundationClaw.ready();
        do {
            clearHubCaches();
            telemetry.addData("Current Position", odometerNavSource.getLocation());
            telemetry.addData("Target Position", navigator.getTargetPosition());
            telemetry.addData("Current Heading", odometerNavSource.getHeading());
            telemetry.addData("Target Heading", navigator.getTargetHeading());
            telemetry.update();
            navigator.update();
        } while (!isStopRequested() && !navigator.atTarget());
        if (isStopRequested()) return;

        foundationClaw.extend();
        TimingUtilities.sleep(this, 0.5, null, null);
        if (isStopRequested()) return;

        navigator.setAllowMovement(false);
        navigator.setTargetHeading(0.5 * Math.PI + Math.toRadians(10));
        do {
            clearHubCaches();
            telemetry.addData("Current Position", odometerNavSource.getLocation());
            telemetry.addData("Target Position", navigator.getTargetPosition());
            telemetry.addData("Current Heading", odometerNavSource.getHeading());
            telemetry.addData("Target Heading", navigator.getTargetHeading());
            telemetry.update();
            navigator.update();
        } while (!isStopRequested() && !pidfMovementStrategy.atHeadingTarget());
        if (isStopRequested()) return;

        //TimingUtilities.sleep(this, 0.2, null, null);
        //if (isStopRequested()) return;

        arm.startControlledArmMotion(0.953);
        odometerNavSource.resetOdometers();

        {
            ElapsedTime timer = new ElapsedTime();
            navigator.setTargetPosition(new Vector2(-75, 570));
            navigator.setAllowMovement(true);
            do {
                clearHubCaches();
                telemetry.addData("Current Position", odometerNavSource.getLocation());
                telemetry.addData("Target Position", navigator.getTargetPosition());
                telemetry.addData("Current Heading", odometerNavSource.getHeading());
                telemetry.addData("Target Heading", navigator.getTargetHeading());
                telemetry.update();
                arm.update();
                navigator.update();
            } while (!isStopRequested() && !navigator.atTarget() && timer.seconds() < 4.3);
            if (isStopRequested()) return;
        }

        arm.release();
        TimingUtilities.runAfterDelay(arm::grab, 1000);

        {
            ElapsedTime timer = new ElapsedTime();
            navigator.setAllowMovement(false);
            navigator.setTargetHeading(Math.PI);
            do {
                clearHubCaches();
                telemetry.addData("Current Position", odometerNavSource.getLocation());
                telemetry.addData("Target Position", navigator.getTargetPosition());
                telemetry.addData("Current Heading", odometerNavSource.getHeading());
                telemetry.addData("Target Heading", navigator.getTargetHeading());
                telemetry.update();
                arm.update();
                navigator.update();
            } while (!isStopRequested() && !pidfMovementStrategy.atHeadingTarget() && timer.seconds() < 1.8);
            if (isStopRequested()) return;
        }
        odometerNavSource.resetOdometers();

        parkingTape.setPower(1);
        TimingUtilities.runAfterDelay(() -> parkingTape.setPower(0), 4000);

        {
            ElapsedTime timer = new ElapsedTime();
            //foundationClaw.retract();
            odometerNavSource.resetOdometers();
            navigator.setAllowMovement(true);
            navigator.setTargetPosition(new Vector2(0, -500));
            do {
                clearHubCaches();
                telemetry.addData("Current Position", odometerNavSource.getLocation());
                telemetry.addData("Target Position", navigator.getTargetPosition());
                telemetry.addData("Current Heading", odometerNavSource.getHeading());
                telemetry.addData("Target Heading", navigator.getTargetHeading());
                telemetry.update();
                arm.update();
                navigator.update();
            } while (!isStopRequested() && !navigator.atTarget() && (timer.seconds() < 0.2 || Math.abs(intake.intake1.getVelocity()) > 8192 * 0.1));
            if (isStopRequested()) return;
        }

        foundationClaw.retract();
        TimingUtilities.sleep(this, 0.75, arm::update, null);
        if (isStopRequested()) return;

        navigator.setTargetPosition(new Vector2(-200, 50));
        do {
            clearHubCaches();
            telemetry.addData("Current Position", odometerNavSource.getLocation());
            telemetry.addData("Target Position", navigator.getTargetPosition());
            telemetry.addData("Current Heading", odometerNavSource.getHeading());
            telemetry.addData("Target Heading", navigator.getTargetHeading());
            telemetry.update();
            arm.update();
            navigator.update();
        } while (!isStopRequested() && !navigator.atTarget());
        if (isStopRequested()) return;

        rb.drive(0, 0, 0);
        TimingUtilities.blockUntil(this, this::isStopRequested, () -> {
            clearHubCaches();
            telemetry.addData("Current Position", odometerNavSource.getLocation());
            telemetry.addData("Target Position", navigator.getTargetPosition());
            telemetry.addData("Current Heading", odometerNavSource.getHeading());
            telemetry.addData("Target Heading", navigator.getTargetHeading());
            arm.update();
            telemetry.update();
        }, null);
    }

    private void clearHubCaches() {
        for (LynxModule revHub : revHubs) {
            revHub.clearBulkCache();
        }
    }

    private boolean navigatorWithin(double distance) {
        return navigator.getTargetPosition().subtract(odometerNavSource.getLocation()).getMagnitude() < distance;
    }
}
