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
import org.ftc9974.thorcore.robot.StallDetector;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;
import org.ftc9974.thorcore.util.CompositeFunction;
import org.ftc9974.thorcore.util.MathUtilities;
import org.ftc9974.thorcore.util.StringUtilities;
import org.ftc9974.thorcore.util.TimingUtilities;

@Autonomous(name = "Skystone Auto")
public class SkystoneAuto extends LinearOpMode {

    private MecanumDrive rb;
    private SensorFusionNavStrategy fusion;
    private Navigator navigator;
    private FoundationClaw claw;

    @Override
    public void runOpMode() throws InterruptedException {
        rb = new MecanumDrive(hardwareMap);
        rb.setAxisInversion(true, true, false);
        telemetry.log().add("Drivetrain Online.");
        telemetry.update();

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
                Math.PI / 2,
                new MecanumEncoderCalculator(13.7, 96)
        );
        fusion.setVuforiaEnabled(false);
        fusion.setSpeedLimit(0);
        telemetry.log().add("Sensor Fusion Online.");
        telemetry.update();
        navigator = new Navigator(fusion, rb, fusion);
        navigator.setEnabled(false);
        navigator.setAllowMovement(true);
        navigator.setAllowTurning(true);
        telemetry.log().add("Navigation Online.");
        telemetry.update();

        claw = new FoundationClaw(hardwareMap);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Ready.");
            telemetry.update();
        }

        claw.setClosedLoopEnabled(false);
        claw.setPower(0.3);

        TimingUtilities.blockUntil(this, ((CompositeFunction) claw::isStalled).withMinimumTime(0.2), this::update, null);
        claw.setPower(0);
        if (isStopRequested()) return;

        claw.homeEncoder();
        claw.retract();
        claw.setClosedLoopEnabled(true);

        navigator.setTargetPosition(new Vector2(100, 0));
        navigator.setEnabled(true);
        TimingUtilities.blockUntil(this, navigator::atTarget, this::update, this::shutdown);
        if (isStopRequested()) return;

        TimingUtilities.blockUntil(this, this::isStopRequested, this::update, this::shutdown);
        this.shutdown();
    }

    private void update() {
        claw.update();
        SensorFusionNavStrategy.DiagnosticData diagnosticData = fusion.getDiagnosticData();
        telemetry.addData("State", diagnosticData.state);
        telemetry.addData("Encoder Targets", StringUtilities.join(", ", diagnosticData.encTargets));
        telemetry.addData("Encoder Positions", StringUtilities.join(", ", rb.getEncoderPositions()));
        telemetry.addData("Encoder Progress", StringUtilities.join(", ", diagnosticData.progress));
        telemetry.addData("Encoder Errors", StringUtilities.join(", ", diagnosticData.errors));
        telemetry.addData("Average Progress", diagnosticData.encProgress);
        //telemetry.addData("To Target", diagnosticData.toTarget.toString());
        telemetry.update();
    }

    private void shutdown() {
        navigator.shutdown();
    }
}
