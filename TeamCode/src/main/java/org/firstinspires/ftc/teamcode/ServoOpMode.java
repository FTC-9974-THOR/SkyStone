package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Op Mode")
public class ServoOpMode extends OpMode {

    Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.servo.get("servo");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            servo.setPosition(1);
        } else {
            servo.setPosition(0);
        }
    }
}
