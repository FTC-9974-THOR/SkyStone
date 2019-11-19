package org.firstinspires.ftc.teamcode.beuler;

import android.os.SystemClock;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Fusion2;
import org.firstinspires.ftc.teamcode.VuforiaKey;
import org.ftc9974.thorcore.control.PIDF;
import org.ftc9974.thorcore.control.RunningAverageFilter;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.control.navigation.VuMarkNavSource;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;
import org.ftc9974.thorcore.util.CompositeFunction;
import org.ftc9974.thorcore.util.TimingUtilities;

@Autonomous(name = "SkyStone")
public class SkyStoneAuto extends LinearOpMode {

    private enum StonePosition {
        LEFT,
        CENTER,
        RIGHT
    }

    private MecanumDrive rb;
    private Arm arm;
    private Fusion2 fusion2;
    private VuMarkNavSource vuforia;
    private Intake intake;
    private FoundationClaw claw;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();
        rb = new MecanumDrive(hardwareMap);
        rb.setAxisInversion(true, true, false);
        rb.setEncoderInversion(false, false, true, true);
        rb.resetEncoders();

        arm = new Arm(hardwareMap);

        fusion2 = new Fusion2(hardwareMap, rb);

        if (fusion2.getNavSource().isFallback()) {
            telemetry.log().add("Warning: Primary IMU failure, using fallback!");
            telemetry.update();
        }

        OpenGLMatrix phoneLocation = OpenGLMatrix
                .translation(0, 0, 0)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES, 90, 0, 0));

        vuforia = new VuMarkNavSource(hardwareMap.appContext, VuforiaKey.KEY, VuforiaLocalizer.CameraDirection.BACK, phoneLocation);

        intake = new Intake(hardwareMap);

        claw = new FoundationClaw(hardwareMap);

        while (!isStopRequested() && !isStarted()) {
            telemetry.addLine("Ready.");
            telemetry.update();
        }
        if (isStopRequested()) {
            return;
        }

        claw.setClosedLoopEnabled(false);
        claw.setPower(0.3);

        TimingUtilities.blockUntil(this, ((CompositeFunction) claw::isStalled).withMinimumTime(0.2), this::update, null);
        claw.setPower(0);
        if (isStopRequested()) return;

        claw.homeEncoder();
        claw.retract();
        claw.setClosedLoopEnabled(true);

        arm.setTargetPosition(3);
        arm.setClosedLoopEnabled(true);

        fusion2.drive(this, new Vector2(0, -350), this::update, 0.7);
        if (isStopRequested()) return;

        if (!vuforia.canSeeStone()) {
            telemetry.log().add("Cannot find stone.");
            telemetry.update();
            TimingUtilities.blockUntil(this, vuforia::canSeeStone, this::update, null);
            if (isStopRequested()) return;
        }

        double stoneOffset;
        {
            OpenGLMatrix stoneLocation = vuforia.getRelativeStoneLocation();
            if (stoneLocation == null) {
                telemetry.log().add("If you see this, something terrible has happened");
                telemetry.update();
                TimingUtilities.blockUntil(this, this::isStopRequested, null, null);
                return;
            }
            VectorF translation = stoneLocation.getTranslation();
            stoneOffset = translation.get(0);
        }

        fusion2.drive(this, new Vector2(0, -200), this::update, 0.7);
        if (isStopRequested()) return;

        double wallDistance = fusion2.getASM().getLeftDistance();

        StonePosition stoneLocation = StonePosition.RIGHT;

        if (wallDistance > 150) {
            telemetry.log().add("Sanity Check failed. Compensating...");
            telemetry.update();
            if (stoneOffset > 100) {
                stoneLocation = StonePosition.LEFT;
            } else if (stoneOffset > -100) {
                stoneLocation = StonePosition.CENTER;
            } else {
                stoneLocation = StonePosition.RIGHT;
            }
        } else {
            telemetry.addData("Vuforia Stone Offset", stoneOffset);
            stoneOffset = 10 * fusion2.getASM().getLeftDistance() + stoneOffset;
            if (stoneOffset > 850) {
                stoneLocation = StonePosition.LEFT;
            } else if (stoneOffset > 650) {
                stoneLocation = StonePosition.CENTER;
            } else {
                stoneLocation = StonePosition.RIGHT;
            }
        }

        telemetry.addData("Wall Distance", wallDistance);
        telemetry.addData("Stone Offset", stoneOffset);
        telemetry.addData("Stone Location", stoneLocation);
        telemetry.update();

        if (stoneLocation != StonePosition.LEFT) {
            fusion2.turnToHeading(this, 0.5 * Math.PI, this::update);
            if (isStopRequested()) return;

            if (stoneLocation == StonePosition.CENTER) {
                fusion2.driveToWallDistance(this, 900, this::update);
            } else {
                fusion2.driveToWallDistance(this, 760, this::update);
            }
            if (isStopRequested()) return;

            arm.release();

            fusion2.drive(this, new Vector2(-200, 0), this::update, 0.5);
            if (isStopRequested()) return;

            rb.drive(-0.3, 0, 0);
            TimingUtilities.blockUntil(this, () -> !(Double.isNaN(fusion2.getASM().getWall0Distance()) && Double.isNaN(fusion2.getASM().getWall1Distance())), () -> {
                rb.drive(-0.3, 0, fusion2.headingPid.update());
                update();
            }, null);
            rb.drive(0, 0, 0);
            if (isStopRequested()) return;

            if (!Double.isNaN(fusion2.getASM().getWall0Distance())) {
                fusion2.drive(this, new Vector2(-300, 0), this::update, 0.6);
            } else {
                fusion2.drive(this, new Vector2(300, 0), this::update, 0.6);
            }
            if (isStopRequested()) return;

            intake.intake(1);

            rb.drive(0, 0.5, 0);
            TimingUtilities.blockUntil(this, fusion2.getASM()::isStonePresent, () -> {
                rb.drive(0, 0.5, 0);
                update();
            }, null);
            rb.drive(0, 0, 0);
            if (isStopRequested()) return;

            arm.setTargetPosition(3.3);

            TimingUtilities.runAfterDelay(() -> {
                intake.stop();
            }, 500);

            fusion2.drive(this, new Vector2(0, -100), this::update, 1);
            if (isStopRequested()) return;

            arm.grab();

            fusion2.turnToHeading(this, 0, this::update);
            if (isStopRequested()) return;

            fusion2.drive(this, new Vector2(0, 520), this::update, 0.7);
            if (isStopRequested()) return;
        }

        fusion2.turnToHeading(this, -0.5 * Math.PI, this::update);
        if (isStopRequested()) return;

        fusion2.drive(this, new Vector2(0, 1500), this::update, 1);
        if (isStopRequested()) return;

        arm.setTargetPosition(0.9);

        fusion2.driveToWallDistance(this, 500, this::update);
        if (isStopRequested()) return;

        fusion2.turnToHeading(this, 0, this::update);
        if (isStopRequested()) return;

        fusion2.driveToWallDistance(this, 720, this::update);
        if (isStopRequested()) return;

        arm.release();
        claw.extend();

        TimingUtilities.runAfterDelay(() -> {
            arm.setTargetPosition(3.2);
        }, 250);

        TimingUtilities.sleep(this, 0.75, this::update, null);
        if (isStopRequested()) return;

        fusion2.driveForwardsToWall(this, this::update);
        if (isStopRequested()) return;

        claw.retract();
        fusion2.drive(this, new Vector2(-850, 0), this::update, 1);
        if (isStopRequested()) return;

        fusion2.driveToWallDistance(this, 430, this::update);
        if (isStopRequested()) return;

        fusion2.turnToHeading(this, 0.5 * Math.PI, this::update);
        if (isStopRequested()) return;

        fusion2.drive(this, new Vector2(0, -300), this::update, 1);
        if (isStopRequested()) return;

        fusion2.driveForwardsToTape(this, this::update);

        //fusion2.driveToWallDistance(this, 500, this::update);
        //if (isStopRequested()) return;

        //TimingUtilities.blockUntil(this, this::isStopRequested, this::update, null);
    }

    private void update() {
        arm.update();
        claw.update();
    }
}
