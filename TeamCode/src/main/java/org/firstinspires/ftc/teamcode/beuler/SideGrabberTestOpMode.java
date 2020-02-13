package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Side Grabber Test")
public class SideGrabberTestOpMode extends OpMode {

    private StoneArm stoneArm;

    @Override
    public void init() {
        stoneArm = new StoneArm(hardwareMap);
    }

    @Override
    public void loop() {
        stoneArm.release();
    }
}
