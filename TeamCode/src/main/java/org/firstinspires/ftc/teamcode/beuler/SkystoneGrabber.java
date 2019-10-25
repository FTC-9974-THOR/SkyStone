package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.robot.Motor;
import org.ftc9974.thorcore.util.UpdateLoopHandler;

public class SkystoneGrabber {

    private static final int GRAB_POSITION = 950,
                             RELEASE_POSITION = 1900;
    private static final int EXTENDED_POSITION = -3930,
                             RETRACTED_POSITION = -100;

    @Hardware
    public Motor armMotor;

    @Hardware
    public ServoImplEx grabberServo;

    @Hardware
    public TouchSensor limit;

    private PIDF armPid;
    private boolean closedLoopEnabled;

    public SkystoneGrabber(HardwareMap hw) {
        Realizer.realize(this, hw);
        grabberServo.setPwmRange(new PwmControl.PwmRange(GRAB_POSITION, RELEASE_POSITION));

        armPid = new PIDF(0.2, 0, 0.00001, 0);
        armPid.setNominalOutputForward(0.1);
        armPid.setNominalOutputReverse(-0.1);
        armPid.setPeakOutputForward(0.8);
        armPid.setPeakOutputReverse(-0.8);
        armPid.setAtTargetThreshold(25);
        armPid.setInputFunction(armMotor::getCurrentPosition);
        armPid.setOutputFunction(this::setArmSpeed);
    }

    public void setClosedLoopEnabled(boolean enabled) {
        closedLoopEnabled = enabled;
    }

    public boolean isClosedLoopEnabled() {
        return closedLoopEnabled;
    }

    public boolean homeSwitchPressed() {
        return limit.isPressed();
    }

    public void resetArmEncoder() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setArmSpeed(double speed) {
        armMotor.setPower(speed);
    }

    public void grab() {
        grabberServo.setPosition(0);
    }

    public void release() {
        grabberServo.setPosition(1);
    }

    public void extend() {
        armPid.setSetpoint(EXTENDED_POSITION);
        setClosedLoopEnabled(true);
    }

    public void retract() {
        armPid.setSetpoint(RETRACTED_POSITION);
        setClosedLoopEnabled(true);
    }

    public boolean atTarget() {
        return armPid.atTarget();
    }

    public void update() {
        if (closedLoopEnabled) {
            armPid.update();
        }
    }
}
