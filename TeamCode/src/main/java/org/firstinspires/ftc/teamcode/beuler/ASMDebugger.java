package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Sensor Debugger")
public class ASMDebugger extends OpMode {

    private AutonomousSensorManager asm;

    private double[] hsv;

    @Override
    public void init() {
        asm = new AutonomousSensorManager(hardwareMap);
        hsv = new double[3];
    }

    @Override
    public void loop() {
        telemetry.addData("Distance", asm.getUltrasonicDistance());
        telemetry.addData("On Tape", asm.onTape());
        System.arraycopy(asm.getTapeSensorHSV(), 0, hsv, 0, 3);
        telemetry.addData("Tape H", hsv[0]);
        telemetry.addData("Tape S", hsv[1]);
        telemetry.addData("Tape V", hsv[2]);
    }
}
