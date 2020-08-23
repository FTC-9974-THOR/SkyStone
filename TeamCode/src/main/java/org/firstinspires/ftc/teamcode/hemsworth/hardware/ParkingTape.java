package org.firstinspires.ftc.teamcode.hemsworth.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;

public class ParkingTape {

    @Hardware
    public DcMotorEx motor;

    @Hardware
    public DigitalChannel stopSwitch;

    public ParkingTape(HardwareMap hw) {
        Realizer.realize(this, hw);

        stopSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    public void setTargetPosition(int position) {
        motor.setTargetPosition(position);
    }

    public boolean isStopSwitchPressed() {
        return stopSwitch.getState();
    }

    public void setMotorPower(double power) {
        motor.setPower(power);
    }

    public void setMotorMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
    }

    public void resetEncoder() {
        DcMotor.RunMode currentMode = motor.getMode();
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(currentMode);
    }
}
