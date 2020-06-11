package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Retract Tape", group = "RND")
public class RetractTapeAuto extends OpMode {

    private ParkingTape tape;

    @Override
    public void init() {
        tape = new ParkingTape(hardwareMap);
    }

    @Override
    public void start() {
        tape.setPower(-1);
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        tape.setPower(0);
    }
}
