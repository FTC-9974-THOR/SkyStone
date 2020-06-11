package org.firstinspires.ftc.teamcode.hemsworth;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.control.navigation.IMUNavSource;
import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.robot.MotorType;
import org.ftc9974.thorcore.robot.drivetrains.swerve.SwerveDrive;
import org.ftc9974.thorcore.robot.drivetrains.swerve.SwerveModule;

import java.util.List;

@TeleOp(name = "Hemsworth")
public class HemsworthTeleOp extends OpMode {

    private SwerveDrive rb;
    private IMUNavSource imu;

    private List<LynxModule> lynxModules;

    @Override
    public void init() {
        rb = new SwerveDrive(hardwareMap);
        rb.setTelemetry(telemetry);

        imu = new IMUNavSource(hardwareMap);

        lynxModules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule lynxModule : lynxModules) {
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void start() {
        rb.resetSlewEncoders();
    }

    @Override
    public void loop() {
        for (LynxModule lynxModule : lynxModules) {
            lynxModule.clearBulkCache();
        }

        double xInput = gamepad1.right_stick_x;
        double yInput = -gamepad1.right_stick_y;
        double tInput = gamepad1.left_stick_x;
        telemetry.addData("X", xInput);
        telemetry.addData("Y", yInput);
        telemetry.addData("T", tInput);

        Orientation orientation = imu.getOrientation().toAxesOrder(AxesOrder.XYZ);
        telemetry.addData("iX", orientation.firstAngle);
        telemetry.addData("iY", orientation.secondAngle);
        telemetry.addData("iZ", orientation.thirdAngle);

        rb.drive(xInput, yInput, tInput);
    }
}
