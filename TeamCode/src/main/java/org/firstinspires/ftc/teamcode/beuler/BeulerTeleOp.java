package org.firstinspires.ftc.teamcode.beuler;

import android.os.SystemClock;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.control.navigation.IMUNavSource;
import org.ftc9974.thorcore.control.navigation.NavSource;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;

@TeleOp(name = "Beuler Tele Op")
public class BeulerTeleOp extends OpMode {

    private MecanumDrive rb;
    private Arm arm;
    private Intake intake;
    private FoundationClaw foundationClaw;

    private Blinkin blinkin;
    private StoneArm stoneArm;

    private NavSource navSource;
    private PIDF headingPid;
    private boolean lastTurnInput;
    private boolean turnMaxDetectionActive;
    private double lastTurnError;
    private long lastTimeStamp;
    private long blinkinTimeStamp;

    private boolean homingSequenceComplete;
    private boolean armRTP;

    @Override
    public void init() {
        rb = new MecanumDrive(hardwareMap);
        rb.setAxisInversion(true, true, false);
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);
        foundationClaw = new FoundationClaw(hardwareMap);

        blinkin = new Blinkin(hardwareMap);
        stoneArm = new StoneArm(hardwareMap);
        stoneArm.retract();
        stoneArm.grab();

        arm.setArmTargetPosition(arm.getArmPosition());
        arm.holdCapstone();

        navSource = new IMUNavSource(hardwareMap);
        headingPid = new PIDF(0.7, 0, 0, 0);
        headingPid.setInputFunction(navSource::getHeading);
        headingPid.setAtTargetThreshold(Math.toRadians(1));
        headingPid.setContinuityRange(-Math.PI, Math.PI);
        headingPid.setContinuous(true);
        Thread.currentThread().setPriority(7);
    }

    @Override
    public void loop() {
        long startTime = System.nanoTime();
        lastTimeStamp = startTime;

        //Vector2 driveInput = new Vector2(gamepad1.right_stick_x, -gamepad1.right_stick_y);
        //driveInput = driveInput.rotate(navSource.getHeading());
        boolean currentTurnInput = Math.abs(gamepad1.left_stick_x) > 0.1;
        double currentTurnError = Math.abs(headingPid.getLastError());
        double x = Math.copySign(Math.pow(gamepad1.right_stick_x, 2), gamepad1.right_stick_x);
        double y = Math.copySign(Math.pow(gamepad1.right_stick_y, 2), -gamepad1.right_stick_y);
        double r = Math.copySign(Math.pow(gamepad1.left_stick_x, 2), -gamepad1.left_stick_x);
        telemetry.addData("X", x);
        telemetry.addData("Y", y);
        telemetry.addData("R", r);
        if (currentTurnInput && !lastTurnInput) {
            // begin turning
            rb.drive(x, y, r);
        } else if (currentTurnInput && lastTurnInput) {
            // turning
            rb.drive(x, y, r);
        } else if (lastTurnInput) {
            // end turning
            turnMaxDetectionActive = true;
            headingPid.setSetpoint(navSource.getHeading());
            rb.drive(x, y, r);
        } else {
            // idle
            if (turnMaxDetectionActive) {
                if (currentTurnError < lastTurnError) {
                    headingPid.setSetpoint(navSource.getHeading());
                    turnMaxDetectionActive = false;
                }
            }
            rb.drive(x, y, headingPid.update());
        }
        lastTurnInput = currentTurnInput;
        lastTurnError = currentTurnError;

        telemetry.addData("Drive System Time", (System.nanoTime() - lastTimeStamp) / 1000000.0);
        lastTimeStamp = System.nanoTime();

        if (gamepad2.y) {
            arm.releaseCapstone();
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            blinkinTimeStamp = SystemClock.uptimeMillis();
        } else if (gamepad2.x) {
            arm.holdCapstone();
        } else if (gamepad2.a) {
            arm.grab();
        } else if (gamepad2.b) {
            arm.release();
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
            blinkinTimeStamp = SystemClock.uptimeMillis();
        }

        telemetry.addData("Manipulator Time", (System.nanoTime() - lastTimeStamp) / 1000000.0);
        lastTimeStamp = System.nanoTime();

        if (gamepad1.x) {
            foundationClaw.extend();
        } else if (gamepad1.y) {
            foundationClaw.retract();
        }

        telemetry.addData("Claw Time", (System.nanoTime() - lastTimeStamp) / 1000000.0);
        lastTimeStamp = System.nanoTime();

        if (!homingSequenceComplete && arm.liftAtBottom()) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
            blinkinTimeStamp = SystemClock.uptimeMillis();
            homingSequenceComplete = true;
        }

        if (gamepad2.left_trigger > 0.8) {
            intake.intake(1);
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else if (gamepad2.right_trigger > 0.8) {
            intake.outtake(1);
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else {
            intake.stop();
            if (SystemClock.uptimeMillis() - blinkinTimeStamp > 1000) {
                blinkin.defaultPattern();
            }
        }

        telemetry.addData("Intake Time", (System.nanoTime() - lastTimeStamp) / 1000000.0);
        lastTimeStamp = System.nanoTime();

        /*boolean currentInput = gamepad2.dpad_up || gamepad2.dpad_down;

        if (gamepad2.right_bumper) {
            armRTP = true;
            arm.setArmTargetPosition(1.39);
        } else if (gamepad2.left_bumper) {
            armRTP = true;
            arm.setArmTargetPosition(0.5);
        }

        if (armRTP) {
            arm.setArmClosedLoopEnabled(true);
            if (currentInput) {
                armRTP = false;
            }
        } else {*/
            arm.setArmClosedLoopEnabled(false);
            if (gamepad2.dpad_up) {
                arm.setShoulderPower(1);
            } else if (gamepad2.dpad_down) {
                arm.setShoulderPower(-1);
            } else {
                // just in case
                arm.setShoulderPower(0);
            }
        //}

        telemetry.addData("Shoulder Time", (System.nanoTime() - lastTimeStamp) / 1000000.0);
        //telemetry.addData("Arm Position", arm.getArmPosition());
        //telemetry.addData("RTP Active", armRTP);
        lastTimeStamp = System.nanoTime();

        arm.setLiftPower(-gamepad2.right_stick_y);
        telemetry.addData("Lift Homed", arm.isLiftHomed());
        //telemetry.addData("Lift Position", arm.getLiftPosition());

        //telemetry.addData("Arm Angle", arm.getArmAngle());
        //telemetry.addData("Lift Height", arm.getLiftHeight());
        //telemetry.addData("Manipulator Height", arm.getManipulatorHeight());
        //telemetry.addData("Manipulator Offset", arm.getManipulatorOffset());

        arm.update();
        foundationClaw.update();

        telemetry.addData("Update Time", (System.nanoTime() - lastTimeStamp) / 1000000.0);
        lastTimeStamp = System.nanoTime();

        telemetry.addData("Loop Time", (System.nanoTime() - startTime) / 1000000.0);
    }
}
