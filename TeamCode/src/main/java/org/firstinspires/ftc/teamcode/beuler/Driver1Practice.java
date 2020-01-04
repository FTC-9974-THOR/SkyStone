package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.control.navigation.IMUNavSource;
import org.ftc9974.thorcore.control.navigation.NavSource;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;
import org.ftc9974.thorcore.util.TimingUtilities;

@TeleOp(name = "Driver 1 Practice")
@Disabled
public class Driver1Practice extends OpMode {

    private MecanumDrive rb;
    private Arm arm;
    private Intake intake;
    private FoundationClaw foundationClaw;
    private AutonomousSensorManager asm;

    private boolean intaking, outtaking;

    private NavSource navSource;
    private PIDF headingPid;
    private boolean lastTurnInput;
    private boolean turnMaxDetectionActive;
    private double lastTurnError;
    private long lastTimeStamp;

    @Override
    public void init() {
        rb = new MecanumDrive(hardwareMap);
        rb.setAxisInversion(true, true, false);
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);
        foundationClaw = new FoundationClaw(hardwareMap);
        asm = new AutonomousSensorManager(hardwareMap);

        arm.setArmTargetPosition(arm.getArmPosition());
        arm.holdCapstone();

        navSource = new IMUNavSource(hardwareMap);
        headingPid = new PIDF(0.7, 0, 0, 0);
        headingPid.setInputFunction(navSource::getHeading);
        headingPid.setAtTargetThreshold(Math.toRadians(1));
        headingPid.setContinuityRange(-Math.PI, Math.PI);
        headingPid.setContinuous(true);

        arm.release();
        arm.setArmTargetPosition(3.0);
        arm.setArmClosedLoopEnabled(true);
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

        if (gamepad1.left_bumper) {
            intaking = true;
            outtaking = false;
            intake.intake(1);
        } else if (gamepad1.right_bumper) {
            intaking = false;
            outtaking = true;
            intake.outtake(1);
        }

        // TODO: 1/2/20 Add stone sensor
        boolean stonePresent = true;//asm.isStonePresent();
        if (intaking && stonePresent) {
            intaking = false;
            TimingUtilities.runAfterDelay(intake::stop, 500);
        } else if (outtaking && !stonePresent) {
            outtaking = false;
            TimingUtilities.runAfterDelay(intake::stop, 500);
        }

        /*if (gamepad2.a) {
            arm.grab();
        } else if (gamepad2.b) {
            arm.release();
        }

        telemetry.addData("Manipulator Time", (System.nanoTime() - lastTimeStamp) / 1000000);
        lastTimeStamp = System.nanoTime();

        if (gamepad2.x) {
            arm.holdCapstone();
        } else if (gamepad2.y) {
            arm.configureForWide();
        }

        telemetry.addData("Yaw Time", (System.nanoTime() - lastTimeStamp) / 1000000);
        lastTimeStamp = System.nanoTime();*/

        if (gamepad1.x) {
            foundationClaw.extend();
        } else if (gamepad1.y) {
            foundationClaw.retract();
        }

        telemetry.addData("Claw Time", (System.nanoTime() - lastTimeStamp) / 1000000);
        lastTimeStamp = System.nanoTime();

        /*if (gamepad2.left_trigger > 0.8) {
            intake.intake(1);
        } else if (gamepad2.right_trigger > 0.8) {
            intake.outtake(1);
        } else {
            intake.stop();
        }

        telemetry.addData("Intake Time", (System.nanoTime() - lastTimeStamp) / 1000000);
        lastTimeStamp = System.nanoTime();

        boolean currentInput = gamepad2.dpad_up || gamepad2.dpad_down;

        if (gamepad2.right_bumper) {
            armRTP = true;
            arm.holdCapstone();
            arm.setArmTargetPosition(1.39);
        } else if (gamepad2.left_bumper) {
            armRTP = true;
            arm.holdCapstone();
            arm.setArmTargetPosition(2.97);
        }

        if (armRTP) {
            arm.setArmClosedLoopEnabled(true);
            if (currentInput) {
                armRTP = false;
            }
        } else {
            arm.setArmClosedLoopEnabled(!currentInput);
            if (lastShoulderInput && !currentInput) {
                // let go
                arm.setArmTargetPosition(arm.getArmPosition());
                maxDetectionActive = true;
            } else if (lastShoulderInput) {
                arm.setArmClosedLoopEnabled(false);
                if (gamepad2.dpad_up) {
                    arm.setShoulderPower(1);
                } else if (gamepad2.dpad_down) {
                    arm.setShoulderPower(-1);
                } else {
                    // just in case
                    arm.setShoulderPower(0);
                }
            } else if (maxDetectionActive) {
                double error = Math.abs(arm.lastArmPIDError());
                if (error < lastError) {
                    arm.setArmTargetPosition(arm.getArmPosition());
                    arm.setShoulderPower(0);
                    maxDetectionActive = false;
                }
            }
        }
        lastShoulderInput = currentInput;
        lastError = Math.abs(arm.lastArmPIDError());

        telemetry.addData("Shoulder Time", (System.nanoTime() - lastTimeStamp) / 1000000);
        lastTimeStamp = System.nanoTime();
         */

        arm.update();
        foundationClaw.update();

        telemetry.addData("Update Time", (System.nanoTime() - lastTimeStamp) / 1000000);
        lastTimeStamp = System.nanoTime();

        telemetry.addData("Loop Time", (System.nanoTime() - startTime) / 1000000.0);
    }
}
