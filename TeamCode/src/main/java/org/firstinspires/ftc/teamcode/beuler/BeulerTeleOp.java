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

    private boolean lastShoulderInput;
    private boolean armRTP;
    private boolean maxDetectionActive;
    private double lastError;

    private NavSource navSource;
    private PIDF headingPid;
    private boolean lastTurnInput;
    private boolean turnMaxDetectionActive;
    private double lastTurnError;
    private long lastTimeStamp;
    private long blinkinTimeStamp;

    @Override
    public void init() {
        rb = new MecanumDrive(hardwareMap);
        rb.setAxisInversion(true, true, false);
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);
        foundationClaw = new FoundationClaw(hardwareMap);

        blinkin = new Blinkin(hardwareMap);

        arm.setTargetPosition(arm.getArmPosition());
        arm.holdCapstone();

        navSource = new IMUNavSource(hardwareMap);
        headingPid = new PIDF(0.7, 0, 0, 0);
        headingPid.setInputFunction(navSource::getHeading);
        headingPid.setAtTargetThreshold(Math.toRadians(1));
        headingPid.setContinuityRange(-Math.PI, Math.PI);
        headingPid.setContinuous(true);
    }

    @Override
    public void loop() {
        long startTime = System.nanoTime();
        lastTimeStamp = startTime;

        //Vector2 driveInput = new Vector2(gamepad1.right_stick_x, -gamepad1.right_stick_y);
        //driveInput = driveInput.rotate(navSource.getHeading());
        boolean currentTurnInput = Math.abs(gamepad1.left_stick_x) > 0.1;
        double currentTurnError = Math.abs(headingPid.getLastError());
        if (currentTurnInput && !lastTurnInput) {
            // begin turning
            rb.drive(gamepad1.right_stick_x, -gamepad1.right_stick_y, -gamepad1.left_stick_x);
        } else if (currentTurnInput && lastTurnInput) {
            // turning
            rb.drive(gamepad1.right_stick_x, -gamepad1.right_stick_y, -gamepad1.left_stick_x);
        } else if (lastTurnInput) {
            // end turning
            turnMaxDetectionActive = true;
            headingPid.setSetpoint(navSource.getHeading());
            rb.drive(gamepad1.right_stick_x, -gamepad1.right_stick_y, -gamepad1.left_stick_x);
        } else {
            // idle
            if (turnMaxDetectionActive) {
                if (currentTurnError < lastTurnError) {
                    headingPid.setSetpoint(navSource.getHeading());
                    turnMaxDetectionActive = false;
                }
            }
            rb.drive(gamepad1.right_stick_x, -gamepad1.right_stick_y, headingPid.update());
        }
        lastTurnInput = currentTurnInput;
        lastTurnError = currentTurnError;

        telemetry.addData("Drive System Time", (System.nanoTime() - lastTimeStamp) / 1000000);
        lastTimeStamp = System.nanoTime();

        if (gamepad2.y) {
            arm.releaseCapstone();
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            blinkinTimeStamp = SystemClock.uptimeMillis();
        } else if (gamepad2.a) {
            arm.grab();
        } else if (gamepad2.b) {
            arm.release();
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
            blinkinTimeStamp = SystemClock.uptimeMillis();
        }

        telemetry.addData("Manipulator Time", (System.nanoTime() - lastTimeStamp) / 1000000);
        lastTimeStamp = System.nanoTime();

        telemetry.addData("Yaw Time", (System.nanoTime() - lastTimeStamp) / 1000000);
        lastTimeStamp = System.nanoTime();

        if (gamepad1.x) {
            foundationClaw.extend();
        } else if (gamepad1.y) {
            foundationClaw.retract();
        }

        telemetry.addData("Claw Time", (System.nanoTime() - lastTimeStamp) / 1000000);
        lastTimeStamp = System.nanoTime();

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

        telemetry.addData("Intake Time", (System.nanoTime() - lastTimeStamp) / 1000000);
        lastTimeStamp = System.nanoTime();

        /*boolean currentInput = gamepad2.dpad_up || gamepad2.dpad_down;

        if (gamepad2.right_bumper) {
            armRTP = true;
            arm.setTargetPosition(1.39);
        } else if (gamepad2.left_bumper) {
            armRTP = true;
            arm.setTargetPosition(2.97);
        }

        if (armRTP) {
            arm.setClosedLoopEnabled(true);
            if (currentInput) {
                armRTP = false;
            }
        } else {
            arm.setClosedLoopEnabled(!currentInput);
            if (lastShoulderInput && !currentInput) {
                // let go
                arm.setTargetPosition(arm.getArmPosition());
                maxDetectionActive = true;
            } else if (lastShoulderInput) {*/
                arm.setClosedLoopEnabled(false);
                if (gamepad2.dpad_up) {
                    arm.setShoulderPower(1);
                } else if (gamepad2.dpad_down) {
                    arm.setShoulderPower(-1);
                } else {
                    // just in case
                    arm.setShoulderPower(0);
                }
            /*} else if (maxDetectionActive) {
                double error = Math.abs(arm.lastPIDError());
                if (error < lastError) {
                    arm.setTargetPosition(arm.getArmPosition());
                    arm.setShoulderPower(0);
                    maxDetectionActive = false;
                }
            }
        }
        lastShoulderInput = currentInput;
        lastError = Math.abs(arm.lastPIDError());*/

        telemetry.addData("Shoulder Time", (System.nanoTime() - lastTimeStamp) / 1000000);
        lastTimeStamp = System.nanoTime();

        arm.update();
        foundationClaw.update();

        telemetry.addData("Update Time", (System.nanoTime() - lastTimeStamp) / 1000000);
        lastTimeStamp = System.nanoTime();

        telemetry.addData("Loop Time", (System.nanoTime() - startTime) / 1000000.0);
    }
}
