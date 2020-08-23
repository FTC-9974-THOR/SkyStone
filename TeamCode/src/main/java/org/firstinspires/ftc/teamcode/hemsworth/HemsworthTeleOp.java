package org.firstinspires.ftc.teamcode.hemsworth;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hemsworth.hardware.FoundationHooks;
import org.firstinspires.ftc.teamcode.hemsworth.hardware.Intake;
import org.firstinspires.ftc.teamcode.hemsworth.hardware.Lift;
import org.firstinspires.ftc.teamcode.hemsworth.hardware.Obamatree;
import org.firstinspires.ftc.teamcode.hemsworth.hardware.ParkingTape;
import org.ftc9974.thorcore.control.navigation.IMUNavSource;
import org.ftc9974.thorcore.robot.drivetrains.swerve.SwerveDrive;
import org.ftc9974.thorcore.util.MathUtilities;

import java.util.List;

@TeleOp(name = "Hemsworth")
public class HemsworthTeleOp extends OpMode {

    private SwerveDrive rb;
    private IMUNavSource imu;
    private FoundationHooks foundationHooks;
    private ParkingTape parkingTape;
    private Lift lift;
    private Obamatree obamatree;
    private Intake intake;

    private List<LynxModule> lynxModules;

    private boolean lastTapeRetractCommand,
                    lastTapeSwitchState,
                    lastLiftCommand;

    @Override
    public void init() {
        rb = new SwerveDrive(hardwareMap);
        rb.setTelemetry(telemetry);
        rb.leftModule.setEncoderOffset(1.5);
        rb.rightModule.setEncoderOffset(1.004);
        rb.leftModule.setHighestEncoderValue(1.725);
        rb.rightModule.setHighestEncoderValue(1.633);

        imu = new IMUNavSource(hardwareMap);

        foundationHooks = new FoundationHooks(hardwareMap);
        foundationHooks.retract();

        parkingTape = new ParkingTape(hardwareMap);

        lift = new Lift(hardwareMap);

        obamatree = new Obamatree(
                hardwareMap,
                rb.leftModule.driveMotor,
                rb.rightModule.driveMotor,
                rb.rightModule.slewMotor
        );

        intake = new Intake(hardwareMap);

        lynxModules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule lynxModule : lynxModules) {
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        for (LynxModule lynxModule : lynxModules) {
            lynxModule.clearBulkCache();
        }

        telemetry.addData("Lift Position", lift.getCurrentPosition());

        telemetry.addData("Left Direction", rb.leftModule.getCurrentDirection());
        telemetry.addData("Right Direction", rb.rightModule.getCurrentDirection());

        double xInput = gamepad1.right_stick_x;
        double yInput = -gamepad1.right_stick_y;
        double tInput = gamepad1.left_stick_x;
        telemetry.addData("X", xInput);
        telemetry.addData("Y", yInput);
        telemetry.addData("T", tInput);

        rb.drive(xInput, yInput, tInput);

        Orientation orientation = imu.getOrientation().toAxesOrder(AxesOrder.XYZ);
        telemetry.addData("iX", orientation.firstAngle);
        telemetry.addData("iY", orientation.secondAngle);
        telemetry.addData("iZ", orientation.thirdAngle);

        //rb.drive(xInput, yInput, tInput);

        if (gamepad1.x) {
            foundationHooks.extend();
        } else if (gamepad1.y) {
            foundationHooks.retract();
        }

        if ((parkingTape.isStopSwitchPressed() && !lastTapeSwitchState) || gamepad1.dpad_left) {
            parkingTape.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            parkingTape.setMotorPower(0);
            parkingTape.resetEncoder();
        } else if (gamepad1.left_bumper && !lastTapeRetractCommand) {
            parkingTape.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            parkingTape.setMotorPower(1);
        } else if (gamepad1.right_bumper) {
            parkingTape.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            parkingTape.setMotorPower(-1);
        }

        lastTapeSwitchState = parkingTape.isStopSwitchPressed();
        lastTapeRetractCommand = gamepad1.left_bumper;

        /*boolean currentLiftCommand = Math.abs(gamepad2.left_stick_y) > 0.05;
        if (currentLiftCommand) {
            lift.setPidEnabled(false);
            double power = -gamepad2.left_stick_y + lift.HOLD_POWER;
            telemetry.addData("Power", MathUtilities.constrain(power, -0.2, 1));
            lift.setLiftPower(power);
        } else if (lastLiftCommand && !lift.isLiftAtBottom()) {
            // falling edge of currentLiftCommand
            lift.setTargetPosition(lift.getCurrentPosition());
            lift.setPidEnabled(true);
        }
        telemetry.addData("Lift current position", lift.getCurrentPosition());
        telemetry.addData("Lift target position", lift.getTargetPosition());
        telemetry.addData("Lift at bottom", lift.isLiftAtBottom());
        telemetry.addData("Lift homed", lift.isHomed());
        lastLiftCommand = currentLiftCommand;*/
        lift.setPidEnabled(false);
        lift.setLiftPower(-gamepad2.left_stick_y);

        telemetry.addData("Tape Switch", parkingTape.isStopSwitchPressed());

        if (gamepad1.dpad_up) {
            obamatree.raiseObameter();
        } else if (gamepad1.dpad_down) {
            obamatree.lowerObameter();
        }

        telemetry.addData("Stone present", intake.isStonePresent());
        if (gamepad1.left_trigger > 0.8) {
            intake.setPower(1);
        } else if (gamepad1.right_trigger > 0.8) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        update();
    }

    private void update() {
        obamatree.update();
        lift.update();
    }
}
