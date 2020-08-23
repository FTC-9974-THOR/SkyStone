package org.firstinspires.ftc.teamcode.hemsworth.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;

public class Lift {

    private final int TOP_EXTENT = 1400;

    public final double HOLD_POWER = 0.27;

    @Hardware
    public DcMotorEx leftMotor, rightMotor;

    @Hardware
    public DigitalChannel homeSensor;

    private PIDF pid;
    private boolean pidEnabled;

    private int encoderOffset;
    private boolean homed,
                    lastHomeSensorReading;

    public Lift(HardwareMap hw) {
        Realizer.realize(this, hw);

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        homeSensor.setMode(DigitalChannel.Mode.INPUT);

        pid = new PIDF(
                0.01,
                0,
                0,
                HOLD_POWER
        );
        pidEnabled = false;
    }

    // positive is up
    public void setLiftPower(double power) {
        if (isLiftAtBottom()) {
            power = Math.max(0, power);
        }
        if (homed && getCurrentPosition() > TOP_EXTENT) {
            power = Math.min(HOLD_POWER, power);
        }
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public void setPidEnabled(boolean enabled) {
        pidEnabled = enabled;
    }

    public void setTargetPosition(int position) {
        pid.setSetpoint(position);
    }

    public int getTargetPosition() {
        return (int) pid.getSetpoint();
    }

    public int getCurrentPosition() {
        return -rightMotor.getCurrentPosition() - encoderOffset;
    }

    public boolean isLiftAtBottom() {
        return homeSensor.getState();
    }

    public void resetEncoder() {
        encoderOffset = getCurrentPosition();
    }

    public boolean isHomed() {
        return homed;
    }

    public void update() {
        if (!homed && lastHomeSensorReading && !isLiftAtBottom()) {
            homed = true;
            resetEncoder();
        }
        lastHomeSensorReading = isLiftAtBottom();
        if (pidEnabled) {
            setLiftPower(pid.update(getCurrentPosition()));
        }
    }
}
