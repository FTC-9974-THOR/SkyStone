package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.VuforiaKey;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.control.navigation.MecanumEncoderCalculator;
import org.ftc9974.thorcore.control.navigation.Navigator;
import org.ftc9974.thorcore.control.navigation.SensorFusionNavStrategy;
import org.ftc9974.thorcore.control.navigation.VuMarkNavSource;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;
import org.ftc9974.thorcore.util.MathUtilities;
import org.ftc9974.thorcore.util.TimingUtilities;

@Autonomous(name = "Skystone Auto")
public class SkystoneAuto extends LinearOpMode {

    private MecanumDrive rb;
    private SensorFusionNavStrategy fusion;
    private Navigator navigator;
    private SkystoneGrabber sg;

    @Override
    public void runOpMode() throws InterruptedException {
        rb = new MecanumDrive(hardwareMap);
        rb.setAxisInversion(false, true, false);

        OpenGLMatrix phoneLocation = new OpenGLMatrix();
        phoneLocation.rotate(AxesReference.EXTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES, -90, 90, 90);
        phoneLocation.translate(0, (float) MathUtilities.inchesToMM(5), 0);
        fusion = new SensorFusionNavStrategy(
                hardwareMap,
                VuforiaKey.KEY,
                VuforiaLocalizer.CameraDirection.BACK,
                phoneLocation,
                new PIDFCoefficients(1, 0, 0, 0),
                new PIDFCoefficients(1, 0, 0, 0),
                new PIDFCoefficients(1, 0, 0, 0),
                1,
                1,
                0.05,
                2,
                2,
                0.075,
                new Vector2(0, 0),
                0,
                new MecanumEncoderCalculator(13.7)
        );
        fusion.setVuforiaEnabled(false);
        navigator = new Navigator(fusion, rb, fusion);
        navigator.setEnabled(false);
        navigator.setAllowMovement(true);
        navigator.setAllowTurning(true);

        sg = new SkystoneGrabber(hardwareMap);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Ready.");
            telemetry.update();
        }

        sg.setArmSpeed(0.6);
        sg.setClosedLoopEnabled(false);
        TimingUtilities.blockUntil(this, sg::homeSwitchPressed, this::update, () -> {
            sg.setArmSpeed(0);
        });
        if (isStopRequested()) {
            return;
        }

        sg.setArmSpeed(0);
        sg.resetArmEncoder();

        sg.release();
        sg.extend();
        TimingUtilities.blockUntil(this, sg::atTarget, this::update, () -> {
            sg.setClosedLoopEnabled(false);
            sg.setArmSpeed(0);
        });
        if (isStopRequested()) {
            return;
        }

        sg.grab();

        TimingUtilities.sleep(this, 1, this::update, null);
        if (isStopRequested()) {
            return;
        }

        sg.retract();
    }

    private void update() {
        sg.update();
    }
}
