package org.firstinspires.ftc.teamcode.hemsworth.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.util.MathUtilities;
import org.ftc9974.thorcore.util.TimingUtilities;

import java.util.concurrent.atomic.AtomicBoolean;

public class Obamatree {

    @Hardware
    public ServoImplEx obameter;

    private DcMotorEx leftYEncoder, rightYEncoder, xEncoder;

    private final double OBAMETER_LOWERED = MathUtilities.map(1200, 500, 2500, 0, 1),
                         OBAMETER_RAISED = MathUtilities.map(1375, 500, 2500, 0, 1),
                         OBAMETER_PEAK = MathUtilities.map(1480, 500, 2500, 0, 1),
                         OBAMETER_PEAK_TIME = 0.2;

    private enum ObameterState {
        RAISED,
        LOWERED,
        PEAK
    }
    private ObameterState state;
    private ElapsedTime timer;

    public Obamatree(HardwareMap hw, DcMotorEx leftY, DcMotorEx rightY, DcMotorEx x) {
        Realizer.realize(this, hw);

        obameter.setPwmRange(new PwmControl.PwmRange(
                500,
                2500
        ));
        leftYEncoder = leftY;
        rightYEncoder = rightY;
        xEncoder = x;

        timer = new ElapsedTime();
    }

    public double getY() {
        double averageEncoderCount = (leftYEncoder.getCurrentPosition() +
                rightYEncoder.getCurrentPosition()) / 2.0;
        double averageRotations = averageEncoderCount / 8192;
        return 72 * Math.PI * averageRotations;
    }

    public double getX() {
        return 50 * Math.PI * (xEncoder.getCurrentPosition() / 8192.0);
    }

    public void lowerObameter() {
        state = ObameterState.LOWERED;
        obameter.setPosition(OBAMETER_LOWERED);
    }

    public void raiseObameter() {
        if (state == ObameterState.LOWERED) {
            state = ObameterState.PEAK;
            timer.reset();
            obameter.setPosition(OBAMETER_PEAK);
        }
    }

    public void update() {
        if (state == ObameterState.PEAK && timer.seconds() > OBAMETER_PEAK_TIME) {
            state = ObameterState.RAISED;
            obameter.setPosition(OBAMETER_RAISED);
        }
    }
}
