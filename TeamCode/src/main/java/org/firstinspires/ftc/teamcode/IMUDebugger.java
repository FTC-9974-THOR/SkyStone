package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.ftc9974.thorcore.control.navigation.IMUNavSource;

@TeleOp(name = "IMU Debugger")
//@Disabled
public class IMUDebugger extends OpMode {

    private IMUNavSource imu;

    @Override
    public void init() {
        imu = new IMUNavSource(hardwareMap);
        if (imu.isFallback()) {
            telemetry.log().add("Primary IMU Failure!");
            telemetry.update();
        }
    }

    @Override
    public void init_loop() {
        telemetry.addData("Calibration Status", imu.getCalibrationStatus().toString());
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("Heading", imu.getHeading());
    }
}
