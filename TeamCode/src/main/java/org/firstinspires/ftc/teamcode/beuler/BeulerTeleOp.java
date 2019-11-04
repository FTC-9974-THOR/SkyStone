package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;

@TeleOp(name = "Beuler Tele Op")
public class BeulerTeleOp extends OpMode {

    private MecanumDrive rb;
    private Arm arm;
    private Intake intake;

    private boolean lastShoulderInput;
    private boolean armRTP;
    private boolean maxDetectionActive;
    private double lastError;

    @Override
    public void init() {
        rb = new MecanumDrive(hardwareMap);
        rb.setAxisInversion(false, true, false);
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);

        arm.setTargetPosition(arm.getArmPosition());
    }

    @Override
    public void loop() {
        long startTime = System.nanoTime();
        rb.drive(-gamepad1.right_stick_x, -gamepad1.right_stick_y, -gamepad1.left_stick_x);

        if (gamepad2.a) {
            arm.grab();
        } else if (gamepad2.b) {
            arm.release();
        }

        if (gamepad2.y) {
            arm.configureForTall();
        } else if (gamepad2.x) {
            arm.configureForWide();
        }

        if (gamepad2.left_trigger > 0.8) {
            intake.intake(1);
        } else if (gamepad2.right_trigger > 0.8) {
            intake.outtake(1);
        } else {
            intake.stop();
        }

        boolean currentInput = gamepad2.dpad_up || gamepad2.dpad_down;

        if (gamepad2.right_bumper) {
            armRTP = true;
            arm.configureForTall();
            arm.setTargetPosition(1.39);
        } else if (gamepad2.left_bumper) {
            armRTP = true;
            arm.configureForTall();
            arm.setTargetPosition(2.4);
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
            } else if (lastShoulderInput) {
                arm.setClosedLoopEnabled(false);
                if (gamepad2.dpad_up) {
                    arm.setShoulderPower(0.5);
                } else if (gamepad2.dpad_down) {
                    arm.setShoulderPower(-0.5);
                } else {
                    // just in case
                    arm.setShoulderPower(0);
                }
            }
            if (maxDetectionActive) {
                double error = Math.abs(arm.lastPIDError());
                if (error < lastError) {
                    arm.setTargetPosition(arm.getArmPosition());
                    maxDetectionActive = false;
                }
            }
        }
        lastShoulderInput = currentInput;
        lastError = Math.abs(arm.lastPIDError());

        arm.update();

        telemetry.addData("Position", arm.getArmPosition());
        telemetry.addData("Closed-Loop Active", arm.isClosedLoopEnabled());
        telemetry.addData("Target Position", arm.getTargetPosition());

        telemetry.addData("Loop Time", (System.nanoTime() - startTime) / 1000000.0);
    }
}
