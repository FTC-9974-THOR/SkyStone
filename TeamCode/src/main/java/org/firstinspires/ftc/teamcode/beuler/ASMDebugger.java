package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.ftc9974.thorcore.control.RunningAverageFilter;

@TeleOp(name = "Sensor Debugger")
public class ASMDebugger extends OpMode {

    private AutonomousSensorManager asm;

    private double[] hsv;

    private RunningAverageFilter filter;
    private double lastUltrasonic;

    private Intake intake;
    private Odometer odometer;

    @Override
    public void init() {
        asm = new AutonomousSensorManager(hardwareMap);
        hsv = new double[3];
        filter = new RunningAverageFilter(3);
        lastUltrasonic = asm.getUltrasonicDistance();

        intake = new Intake(hardwareMap);
        odometer = new Odometer(hardwareMap, intake.intake0);
        odometer.resetOdometer();
    }

    @Override
    public void start() {
        odometer.extend();
    }

    @Override
    public void loop() {
        double currentUltrasonic = asm.getUltrasonicDistance();
        telemetry.addData("Distance", currentUltrasonic);
        double filteredUltrasonic = filter.update((currentUltrasonic > 1000) ? lastUltrasonic : currentUltrasonic);
        telemetry.addData("Filtered Distance", filteredUltrasonic);
        telemetry.addData("On Tape", asm.onTape());
        System.arraycopy(asm.getTapeSensorHSV(), 0, hsv, 0, 3);
        telemetry.addData("Tape H", hsv[0]);
        telemetry.addData("Tape S", hsv[1]);
        telemetry.addData("Tape V", hsv[2]);
        lastUltrasonic = currentUltrasonic;
        if (gamepad1.a) {
            odometer.extend();
        } else if (gamepad1.b) {
            odometer.retract();
        }
        telemetry.addData("Odometer Position", odometer.getOdometerPosition());
    }

    @Override
    public void stop() {
        odometer.retract();
    }
}
