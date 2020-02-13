package org.firstinspires.ftc.teamcode.beuler;

import android.os.SystemClock;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.control.navigation.IMUNavSource;
import org.ftc9974.thorcore.control.navigation.NavSource;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;

@TeleOp(name = "Driver 1 Practice")
public class Driver1Practice extends OpMode {

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
    private boolean armRTP, lastReady, lastPlace, lastRetract;
    private boolean liftRTP;
    private boolean lastLiftInput;
    private boolean liftMaxDetectionActive;
    private double lastLiftError;

    private enum RetractState {
        RELEASE,
        LIFT_A_BIT,
        REVOLVE_ARM,
        LOWER_LIFT_UNTIL_ARM_READY,
        LOWER_LIFT_WITH_ARM
    }
    private RetractState retractState;
    private ElapsedTime retractTimer;
    private boolean retractSequenceEngaged;

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

        retractTimer = new ElapsedTime();
    }

    @Override
    public void loop() {
        long startTime = System.nanoTime();
        lastTimeStamp = startTime;

        //Vector2 driveInput = new Vector2(gamepad1.right_stick_x, -gamepad1.right_stick_y);
        //driveInput = driveInput.rotate(navSource.getHeading());
        boolean currentTurnInput = Math.abs(gamepad1.left_stick_x) > 0.1;
        double currentTurnError = Math.abs(headingPid.getLastError());
        double x = gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x);
        double y = -gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y);
        double r = -gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);
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

        if (gamepad1.start) {
            headingPid.setSetpoint(navSource.getHeading());
        }

        telemetry.addData("Drive System Time", (System.nanoTime() - lastTimeStamp) / 1000000.0);
        lastTimeStamp = System.nanoTime();

        if (gamepad1.y) {
            arm.releaseCapstone();
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            blinkinTimeStamp = SystemClock.uptimeMillis();
        } else if (gamepad1.x) {
            arm.holdCapstone();
        } else if (gamepad1.a) {
            arm.grab();
        } else if (gamepad1.b) {
            arm.release();
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
            blinkinTimeStamp = SystemClock.uptimeMillis();
        }

        telemetry.addData("Manipulator Time", (System.nanoTime() - lastTimeStamp) / 1000000.0);
        lastTimeStamp = System.nanoTime();

        /*if (gamepad1.x) {
            foundationClaw.extend();
        } else if (gamepad1.y) {
            foundationClaw.retract();
        }

        telemetry.addData("Claw Time", (System.nanoTime() - lastTimeStamp) / 1000000.0);
        lastTimeStamp = System.nanoTime();*/

        if (!homingSequenceComplete && arm.liftAtBottom()) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
            blinkinTimeStamp = SystemClock.uptimeMillis();
            homingSequenceComplete = true;
        }

        if (gamepad1.left_trigger > 0.8 || gamepad2.left_trigger > 0.8) {
            intake.intake(1);
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else if (gamepad1.right_trigger > 0.8 || gamepad2.right_trigger > 0.8) {
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

        if (gamepad1.back && !lastRetract) {
            retractState = RetractState.RELEASE;
            retractTimer.reset();
            arm.release();
            retractSequenceEngaged = true;
        }
        lastRetract = gamepad1.back;

        if (retractSequenceEngaged) {
            switch (retractState) {
                case RELEASE:
                    if (retractTimer.seconds() > 0.3) {
                        retractState = RetractState.LIFT_A_BIT;
                        liftRTP = true;
                        arm.setLiftTargetPosition(Math.min(arm.getLiftPosition() + 900, arm.getLiftMaxPosition()));
                    }
                    break;
                case LIFT_A_BIT:
                    if (Math.abs(arm.lastLiftPIDError()) < 75) {
                        armRTP = true;
                        arm.startControlledArmMotion(1.7);
                        retractTimer.reset();
                        retractState = RetractState.REVOLVE_ARM;
                    }
                    break;
                case REVOLVE_ARM:
                    if (arm.getArmPosition() > 1.5) {
                        liftRTP = true;
                        arm.setLiftTargetPosition(200);
                        retractState = RetractState.LOWER_LIFT_UNTIL_ARM_READY;
                    }
                    break;
                case LOWER_LIFT_UNTIL_ARM_READY:
                    if (retractTimer.seconds() > 1.5) {
                        armRTP = true;
                        arm.startControlledArmMotion(3);
                        retractState = RetractState.LOWER_LIFT_WITH_ARM;
                        arm.setLiftClosedLoopEnabled(false);
                        arm.setLiftPower(-1);
                    }
                    break;
                case LOWER_LIFT_WITH_ARM:
                    if (arm.getLiftPosition() < 0 || arm.liftAtBottom()) {
                        arm.setLiftPower(0);
                    }
                    if (arm.getLiftPosition() < 230 && arm.getArmPosition() > 2.8) {
                        retractSequenceEngaged = false;
                        liftRTP = false;
                        armRTP = false;
                    }
                    break;
            }
        }

        telemetry.addData("Retract Sequence Engaged", retractSequenceEngaged);
        telemetry.addData("Retract State", retractState);

        boolean currentInput = gamepad1.dpad_up || gamepad1.dpad_down || Math.abs(gamepad2.left_stick_y) > 0.05;

        if (gamepad1.left_bumper) {
            if (!lastReady) {
                arm.startControlledArmMotion(1.45);
            }
            armRTP = true;
        } else if (gamepad1.right_bumper) {
            if (!lastPlace) {
                arm.startControlledArmMotion(0.953);
            }
            armRTP = true;
        }
        lastReady = gamepad1.left_bumper;
        lastPlace = gamepad1.right_bumper;

        if (armRTP) {
            if (currentInput) {
                retractSequenceEngaged = false;
                arm.stopControlledArmMotion();
                armRTP = false;
            }
        } else {
            arm.setArmClosedLoopEnabled(false);
            if (Math.abs(gamepad2.left_stick_y) > 0.05) {
                arm.setShoulderPower(-gamepad2.left_stick_y);
            } else if (gamepad1.dpad_up) {
                arm.setShoulderPower(1);
            } else if (gamepad1.dpad_down) {
                arm.setShoulderPower(-1);
            } else {
                // just in case
                arm.setShoulderPower(0);
            }
        }

        telemetry.addData("Stick", gamepad1.left_stick_y);
        telemetry.addData("Speed Limit", arm.speedLimit);
        telemetry.addData("Arm RTP", armRTP);
        telemetry.addData("Start Point", arm.shoulderStartPoint);
        telemetry.addData("Target Point", arm.getArmTargetPosition());
        telemetry.addData("To Target", arm.toTarget);
        telemetry.addData("From Start", arm.fromStart);
        telemetry.addData("Arm Position", arm.getArmPosition());
        telemetry.addData("Last Arm Error", arm.lastArmPIDError());
        telemetry.addData("Pid Output", arm.pidOutput);
        telemetry.addData("Shoulder Time", (System.nanoTime() - lastTimeStamp) / 1000000.0);
        //telemetry.addData("Arm Position", arm.getArmPosition());
        //telemetry.addData("RTP Active", armRTP);
        lastTimeStamp = System.nanoTime();

        boolean liftInput = Math.abs(gamepad2.right_stick_y) > 0.05;
        if (gamepad2.left_bumper) {
            liftRTP = true;
            arm.setLiftTargetPosition(arm.getLiftMaxPosition());
        }

        if (liftRTP) {
            arm.setLiftClosedLoopEnabled(true);
            if (liftInput) {
                retractSequenceEngaged = false;
                liftRTP = false;
            }
        } else {
            if (liftInput || !arm.isLiftHomed()) {
                arm.setLiftClosedLoopEnabled(false);
                arm.setLiftPower(-gamepad2.right_stick_y);
            } else if (lastLiftInput) {
                arm.setLiftClosedLoopEnabled(true);
                arm.setLiftTargetPosition(arm.getLiftPosition());
                liftMaxDetectionActive = true;
                lastLiftError = -1;
            }

            if (liftMaxDetectionActive) {
                double currentError = Math.abs(arm.lastLiftPIDError());
                if (currentError < lastLiftError) {
                    liftMaxDetectionActive = false;
                    arm.setLiftTargetPosition(arm.getLiftPosition());
                }
                lastLiftError = currentError;
            }
        }
        lastLiftInput = liftInput;

        telemetry.addData("Lift RTP", liftRTP);
        telemetry.addData("Lift Target", arm.getLiftTargetPosition());
        telemetry.addData("Lift Homed", arm.isLiftHomed());
        telemetry.addData("Lift Position", arm.getLiftPosition());

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
