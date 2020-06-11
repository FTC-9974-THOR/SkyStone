package org.firstinspires.ftc.teamcode.hemsworth;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.ftc9974.thorcore.robot.drivetrains.swerve.SwerveModule;

@TeleOp(name = "MA3 Test")
public class MA3EncoderTestOpMode extends OpMode {

    private SwerveModule module;

    @Override
    public void init() {
        module = new SwerveModule("left", hardwareMap);
        module.setUseQuadEncoder(false);
        module.setEncoderOffset(3.292);
    }

    @Override
    public void loop() {
        telemetry.addData("Current Direction", module.getCurrentDirection());
    }
}
