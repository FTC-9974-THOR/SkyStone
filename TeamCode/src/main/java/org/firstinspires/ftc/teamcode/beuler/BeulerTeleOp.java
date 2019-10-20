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

    @Override
    public void init() {
        rb = new MecanumDrive(hardwareMap);
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);

        arm.setTargetPosition(arm.getArmPosition());
    }

    @Override
    public void loop() {
        rb.drive(-gamepad1.right_stick_x, -gamepad1.right_stick_y, -gamepad1.left_stick_x);
        if (gamepad1.a) {
            arm.grab();
        } else if (gamepad1.b) {
            arm.release();
        }

        if (gamepad1.y) {
            arm.configureForTall();
        } else if (gamepad1.x) {
            arm.configureForWide();
        }

        if (gamepad1.left_trigger > 0.8) {
            intake.intake(1);
        } else if (gamepad1.right_trigger > 0.8) {
            intake.outtake(1);
        } else {
            intake.stop();
        }

        boolean currentInput = gamepad1.dpad_up || gamepad1.dpad_down;

        if (gamepad1.left_bumper) {
            armRTP = true;
            arm.setTargetPosition(2.26);
        }

        if (armRTP) {
            arm.setClosedLoopEnabled(true);
            if (currentInput) {
                armRTP = false;
            }
        } else {
            arm.setClosedLoopEnabled(!currentInput);
            if (lastShoulderInput && !currentInput) {
                arm.setTargetPosition(arm.getArmPosition());
            } else if (lastShoulderInput) {
                arm.setClosedLoopEnabled(false);
                if (gamepad1.dpad_up) {
                    arm.setShoulderPower(0.5);
                } else if (gamepad1.dpad_down) {
                    arm.setShoulderPower(-0.5);
                }
            }
        }
        lastShoulderInput = currentInput;

        telemetry.addData("Position", arm.getArmPosition());
        telemetry.addData("Closed-Loop Active", arm.isClosedLoopEnabled());
        telemetry.addData("Target Position", arm.getTargetPosition());
    }
}
