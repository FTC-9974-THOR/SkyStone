package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Fusion2;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;
import org.ftc9974.thorcore.util.CompositeFunction;
import org.ftc9974.thorcore.util.TimingUtilities;

@Autonomous(name = "Red Foundation Auto", group = "Foundation")
public class BlueFoundationAuto extends LinearOpMode {

    private MecanumDrive rb;
    private Arm arm;
    private FoundationClaw claw;
    private Fusion2 fusion2;
    //private VuMarkNavSource vuforia;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();
        rb = new MecanumDrive(hardwareMap);
        rb.setAxisInversion(true, true, false);
        rb.setEncoderInversion(false, false, true, true);
        rb.resetEncoders();

        arm = new Arm(hardwareMap);

        claw = new FoundationClaw(hardwareMap);

        fusion2 = new Fusion2(hardwareMap, rb);

        /*OpenGLMatrix phoneLocation = OpenGLMatrix
                .translation(0, 0, 0)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES, 90, 0, 0));

        vuforia = new VuMarkNavSource(hardwareMap.appContext, VuforiaKey.KEY, VuforiaLocalizer.CameraDirection.BACK, phoneLocation);
         */

        while (!isStopRequested() && !isStarted()) {
            telemetry.addLine("Ready.");
            telemetry.update();
        }
        if (isStopRequested()) {
            return;
        }

        // home the claw
        claw.setClosedLoopEnabled(false);
        claw.setPower(0.3);

        TimingUtilities.blockUntil(this, ((CompositeFunction) claw::isStalled).withMinimumTime(0.2), this::update, null);
        claw.setPower(0);
        if (isStopRequested()) return;

        claw.homeEncoder();
        claw.retract();
        claw.setClosedLoopEnabled(true);

        // grab the stone and move the arm up
        arm.grab();

        // drive backwards, 1st segment
        fusion2.drive(this, new Vector2(0, -450), this::update, 0.5);
        if (isStopRequested()) {
            return;
        }

        arm.setTargetPosition(2.79);
        arm.setClosedLoopEnabled(true);

        // strafe right
        fusion2.driveToLeftDistance(this, 35, this::update);
        if (isStopRequested()) {
            return;
        }

        // back up to foundation
        fusion2.drive(this, new Vector2(0, -400), this::update, 0.5);
        if (isStopRequested()) {
            return;
        }

        claw.extend();
        arm.setTargetPosition(0.6);
        arm.setClosedLoopEnabled(true);
        //TimingUtilities.blockUntil(this, arm::shoulderAtTarget, this::update, null);
        if (isStopRequested()) return;

        arm.release();

        TimingUtilities.sleep(this, 0.5, this::update, null);
        if (isStopRequested()) return;

        arm.setTargetPosition(3.2);
        arm.grab();

        fusion2.driveForwardsToWall(this, this::update);
        if (isStopRequested()) return;

        claw.retract();
        TimingUtilities.blockUntil(this, claw::atTarget, this::update, null);
        if (isStopRequested()) return;

        TimingUtilities.sleep(this, 6, this::update, null);
        if (isStopRequested()) return;

        fusion2.drive(this,  new Vector2(930, 0), this::update, 0.7);
        if (isStopRequested()) return;

        fusion2.drive(this, new Vector2(0, -350), this::update, 0.7);
        if (isStopRequested()) return;

        TimingUtilities.sleep(this, 0.5, this::update, null);
        if (isStopRequested()) return;

        fusion2.turnToHeading(this, 0.5 * Math.PI, this::update);
        if (isStopRequested()) return;

        rb.drive(0, 0.5, 0);
        TimingUtilities.sleep(this, 1, this::update, null);
        rb.drive(0, 0, 0);
        if (isStopRequested()) return;

        TimingUtilities.sleep(this, 0.5, this::update, null);
        if (isStopRequested()) return;

        fusion2.drive(this, new Vector2(-35, 0), this::update, 0.5);
        if (isStopRequested()) return;

        arm.setTargetPosition(3.2);
        arm.release();

        //TimingUtilities.sleep(this, 0.5, this::update, null);
        //if (isStopRequested()) return;

        //fusion2.drive(this, new Vector2(0, -200), this::update, 0.5);
        //if (isStopRequested()) return;

        fusion2.driveBackwardsToTape(this, this::update);
        if (isStopRequested()) return;

        //TimingUtilities.blockUntil(this, arm::shoulderAtTarget, this::update, null);
        arm.setShoulderPower(0);
        if (isStopRequested()) return;

        //fusion2.driveToLeftDistance(this, 750, this::update);
        //if (isStopRequested()) return;

        //TimingUtilities.sleep(this, 0.5, this::update, null);
        //if (isStopRequested()) return;

        //telemetry.addData("Stone detected", vuforia.canSeeStone());
        //telemetry.update();

        /*
        {
            boolean seenStones = false;
            double direction = -0.7;
            rb.drive(-0.5, 0, 0);
            while (!isStopRequested()) {
                if (!seenStones && !fusion2.getASM().isSkystone()) {
                    seenStones = true;
                }
                if (vuforia.canSeeStone() || (seenStones && fusion2.getASM().isSkystone())) {
                    break;
                }

                double distance = fusion2.getASM().getLeftDistance();
                if (distance < 57) {
                    direction = 0.2;
                } else if (distance > 105) {
                    direction = -0.2;
                }

                rb.drive(direction, 0, fusion2.headingPid.update());
            }
        }
         */
        rb.drive(0, 0, 0);

        //TimingUtilities.blockUntil(this, this::isStopRequested, this::update, null);
    }

    private void update() {
        arm.update();
        claw.update();
    }
}
