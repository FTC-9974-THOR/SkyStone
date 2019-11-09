package org.firstinspires.ftc.teamcode.beuler;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.VuforiaKey;
import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.control.navigation.IMUNavSource;
import org.ftc9974.thorcore.control.navigation.MecanumEncoderCalculator;
import org.ftc9974.thorcore.control.navigation.Navigator;
import org.ftc9974.thorcore.control.navigation.SensorFusionNavStrategy;
import org.ftc9974.thorcore.control.navigation.VuMarkNavSource;
import org.ftc9974.thorcore.robot.StallDetector;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;
import org.ftc9974.thorcore.util.CompositeFunction;
import org.ftc9974.thorcore.util.LambdaUtilities;
import org.ftc9974.thorcore.util.ListUtilities;
import org.ftc9974.thorcore.util.MathUtilities;
import org.ftc9974.thorcore.util.StringUtilities;
import org.ftc9974.thorcore.util.TimingUtilities;

@Disabled
@Autonomous(name = "Foundation")
public class FoundationAuto extends LinearOpMode {

    private MecanumDrive rb;
    private IMUNavSource navSource;
    private FoundationClaw claw;
    private AutonomousSensorManager asm;
    private PIDF headingPid, strafePid;
    private MecanumEncoderCalculator calculator;

    @Override
    public void runOpMode() throws InterruptedException {
        rb = new MecanumDrive(hardwareMap);
        rb.setAxisInversion(true, true, false);
        rb.setEncoderInversion(false, false, true, true);
        rb.resetEncoders();
        navSource = new IMUNavSource(hardwareMap);
        claw = new FoundationClaw(hardwareMap);
        asm = new AutonomousSensorManager(hardwareMap);
        calculator = new MecanumEncoderCalculator(13.7 * 2.0, 96);

        headingPid = new PIDF(0.3, 0, 0.01, 0);
        headingPid.setPhase(false);
        headingPid.setPeakOutputForward(1);
        headingPid.setPeakOutputReverse(-1);
        strafePid = new PIDF(0.25, 0, 0, 0);
        strafePid.setNominalOutputForward(0.1);
        strafePid.setNominalOutputReverse(-0.1);
        strafePid.setPeakOutputForward(0.5);
        strafePid.setPeakOutputReverse(-0.5);
        strafePid.setPhase(false);
        strafePid.setAtTargetThreshold(1);
        strafePid.setInputFunction(asm::getLeftDistance);
        headingPid.setInputFunction(navSource::getHeading);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Ready.");
            update();
            telemetry.update();
        }
        if (isStopRequested()) {
            shutdown();
            return;
        }

        claw.setClosedLoopEnabled(false);
        claw.setPower(0.3);

        TimingUtilities.blockUntil(this, ((CompositeFunction) claw::isStalled).withMinimumTime(0.2), this::update, null);
        claw.setPower(0);
        if (isStopRequested()) return;

        claw.homeEncoder();
        claw.extend();
        claw.setClosedLoopEnabled(true);

        headingPid.setSetpoint(0);

        {
            rb.resetEncoders();
            Vector2 targetPosition = new Vector2(0, MathUtilities.inchesToMM(-28));
            int[] targets = calculator.calculate(targetPosition);
            Vector2 normalizedTarget = targetPosition.scalarDivide(targetPosition.getMagnitude());
            rb.drive(normalizedTarget.getX(), normalizedTarget.getY(), headingPid.update());
            int[] errors = new int[4];
            double[] progress = new double[4];
            while (!isStopRequested()) {
                int[] positions = rb.getEncoderPositions();
                for (int i = 0; i < targets.length; i++) {
                    errors[i] = Math.abs(targets[i] - positions[i]);
                    progress[i] = (double) positions[i] / (double) targets[i];
                }
                double avgProgress = MathUtilities.average(progress);
                if (MathUtilities.min(errors) < 50 || avgProgress > 1) {
                    break;
                }
                double speed = 0.7;
                double nominalSpeed = 0.1;
                double slowdownPoint = 0.25;
                if (avgProgress > slowdownPoint) {
                    speed = MathUtilities.map(avgProgress, slowdownPoint, 1, speed, nominalSpeed);
                    if (speed < nominalSpeed) {
                        speed = nominalSpeed;
                    }
                }
                rb.drive(speed * normalizedTarget.getX(), speed * normalizedTarget.getY(), headingPid.update());
                telemetry.addData("Progress", avgProgress);
                telemetry.update();
            }
            rb.drive(0, 0, 0);
            if (isStopRequested()) return;
        }
        TimingUtilities.sleep(this, 0.5, this::update, this::shutdown);
        if (isStopRequested()) return;

        {
            boolean raisedClaw = false;
            long clawStart = SystemClock.uptimeMillis() + 1100;
            double target = 26;
            while (!isStopRequested()) {
                double distance = target - asm.getLeftDistance();

                if (SystemClock.uptimeMillis() > clawStart /*asm.getLeftDistance() < 67*/ && !raisedClaw) {
                    rb.drive(-0.3, 0, 0);
                    claw.retract();
                    TimingUtilities.blockUntil(this, ((CompositeFunction) claw::atTarget).or(claw::isStalled), this::update, this::shutdown);
                    TimingUtilities.sleep(this, 0.25, null, null);
                    if (isStopRequested()) return;
                    raisedClaw = true;
                }

                if (Math.abs(distance) < 10 || distance > 0) {
                    break;
                }
                double speed = -0.7;
                double nominalSpeed = 0.2;
                double slowdownPoint = 10;
                if (Math.abs(distance) < slowdownPoint) {
                    speed = MathUtilities.map(distance, slowdownPoint, 0, speed, Math.copySign(nominalSpeed, speed));
                    if (Math.abs(speed) < nominalSpeed) {
                        speed = Math.copySign(nominalSpeed, speed);
                    }
                }
                rb.drive(speed, 0, headingPid.update());
                telemetry.addData("Distance", distance);
                telemetry.update();
                update();
            }
            rb.drive(0, -0.1, 0);
            if (isStopRequested()) return;
        }

        TimingUtilities.sleep(this, 0.3, this::update, this::shutdown);
        if (isStopRequested()) return;


        claw.extend();
        TimingUtilities.blockUntil(this, ((CompositeFunction) claw::atTarget).or(claw::isStalled), this::update, this::shutdown);
        TimingUtilities.sleep(this, 0.5, null, null);
        if (isStopRequested()) return;

        {
            rb.resetEncoders();
            long startTime = SystemClock.uptimeMillis();
            long timeout = 5000;
            Vector2 targetPosition = new Vector2(0, MathUtilities.inchesToMM(39.5));
            int[] targets = calculator.calculate(targetPosition);
            Vector2 normalizedTarget = targetPosition.scalarDivide(targetPosition.getMagnitude());
            rb.drive(normalizedTarget.getX(), normalizedTarget.getY(), headingPid.update());
            int[] errors = new int[4];
            double[] progress = new double[4];
            while (!isStopRequested() && SystemClock.uptimeMillis() - startTime < timeout) {
                int[] positions = rb.getEncoderPositions();
                for (int i = 0; i < targets.length; i++) {
                    errors[i] = Math.abs(targets[i] - positions[i]);
                    progress[i] = (double) positions[i] / (double) targets[i];
                }
                double avgProgress = MathUtilities.average(progress);
                if (MathUtilities.min(errors) < 50 || avgProgress > 1 || avgProgress > 0.82) {
                    break;
                }
                double speed = 0.7;
                double nominalSpeed = 0.4;
                double slowdownPoint = 0.25;
                if (avgProgress > slowdownPoint) {
                    speed = MathUtilities.map(avgProgress, slowdownPoint, 1, speed, nominalSpeed);
                    if (speed < nominalSpeed) {
                        speed = nominalSpeed;
                    }
                }
                rb.drive(speed * normalizedTarget.getX(), speed * normalizedTarget.getY(), headingPid.update());
                telemetry.addData("Progress", avgProgress);
                telemetry.addData("Time Elapsed", SystemClock.uptimeMillis() - startTime);
                telemetry.update();
            }
            rb.drive(0, 0, 0);
            if (isStopRequested()) return;
        }

        claw.retract();
        TimingUtilities.blockUntil(this, ((CompositeFunction) claw::atTarget).or(claw::isStalled), this::update, this::shutdown);
        if (isStopRequested()) return;

        {
            headingPid.setSetpoint(0);
            rb.drive(0.5, 0, 0);
            TimingUtilities.blockUntil(this, asm::onTape, () -> {
                double strafeSpeed = 0.5;
                if (asm.getLeftDistance() > 40) {
                    strafeSpeed = 0.3;
                }
                update();
                rb.drive(strafeSpeed, 0, headingPid.update());
            }, this::shutdown);
            if (isStopRequested()) return;
            rb.drive(0, 0, 0);
        }

        //TimingUtilities.blockUntil(this, this::isStopRequested, this::update, this::shutdown);
        shutdown();
    }

    private void update() {
        claw.update();
        telemetry.addData("Heading", navSource.getHeading());
        telemetry.update();
    }

    private void shutdown() {
        //navigator.shutdown();
    }
}
