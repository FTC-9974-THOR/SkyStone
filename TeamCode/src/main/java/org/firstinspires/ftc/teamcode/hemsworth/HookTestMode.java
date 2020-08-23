package org.firstinspires.ftc.teamcode.hemsworth;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hemsworth.hardware.FoundationHooks;
import org.firstinspires.ftc.teamcode.hemsworth.hardware.ParkingTape;

@TeleOp(name = "Hook Test Mode")
public class HookTestMode extends OpMode {

    private FoundationHooks foundationHooks;
    private ParkingTape parkingTape;

    @Override
    public void init() {
        foundationHooks = new FoundationHooks(hardwareMap);
        foundationHooks.retract();

        parkingTape = new ParkingTape(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            foundationHooks.extend();
        } else if (gamepad1.y) {
            foundationHooks.retract();
        }

        if (gamepad1.left_bumper) {
            parkingTape.setTargetPosition(0);
        } else if (gamepad1.right_bumper) {
            parkingTape.setTargetPosition(-1000);
        }
    }
}
