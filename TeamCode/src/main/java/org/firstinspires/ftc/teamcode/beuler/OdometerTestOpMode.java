package org.firstinspires.ftc.teamcode.beuler;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.control.navigation.Navigator;
import org.ftc9974.thorcore.control.navigation.OdometerNavSource;
import org.ftc9974.thorcore.control.navigation.PIDFMovementStrategy;
import org.ftc9974.thorcore.control.navigation.SynchronousNavigator;
import org.ftc9974.thorcore.robot.drivetrains.MecanumDrive;
import org.ftc9974.thorcore.util.MathUtilities;

import java.util.List;

@TeleOp(name = "Odometer Test", group = "RND")
public class OdometerTestOpMode extends OpMode {

    private OdometerNavSource odometerNavSource;
    private PIDFMovementStrategy pidfMovementStrategy;
    private SynchronousNavigator navigator;
    private MecanumDrive rb;

    private List<LynxModule> revHubs;

    @Override
    public void init() {
        odometerNavSource = new OdometerNavSource(
                hardwareMap,
                hardwareMap.get(DcMotorEx.class, "I-intake0"),
                hardwareMap.get(DcMotorEx.class, "I-intake1"),
                new Vector2(MathUtilities.inchesToMM(-9) + 25, MathUtilities.inchesToMM(-9) + 155),
                new Vector2(MathUtilities.inchesToMM(9) - 25, MathUtilities.inchesToMM(9) - 188),
                50.8,
                8192);
        odometerNavSource.setXInversion(true);
        odometerNavSource.setYInversion(true);
        pidfMovementStrategy = new PIDFMovementStrategy(
                0.008, 0, 0.0004, 0,
                0.008, 0, 0.0004, 0,
                1, 0, 0, 0,
                0.2,
                0.2,
                0.2,
                8,
                8,
                0.02
        );
        pidfMovementStrategy.setContinuity(-Math.PI, Math.PI);
        rb = new MecanumDrive(hardwareMap);
        rb.setAxisInversion(true, true, false);
        rb.setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        navigator = new SynchronousNavigator(odometerNavSource, rb, pidfMovementStrategy);
        navigator.setEnabled(false);
        revHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule revHub : revHubs) {
            revHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        ServoImplEx gearServo = hardwareMap.get(ServoImplEx.class, "O-gearServo");
        gearServo.setPwmRange(new PwmControl.PwmRange(1875, 2160));
        gearServo.setPosition(1);
    }

    @Override
    public void start() {
        odometerNavSource.resetOdometers();
        navigator.setTargetPosition(new Vector2(300, 200));
        navigator.setTargetHeading(0);
        navigator.setEnabled(true);
    }

    @Override
    public void loop() {
        for (LynxModule revHub : revHubs) {
            revHub.clearBulkCache();
        }
        navigator.update();
        //telemetry.addData("IMU Heading", odometerNavSource.getHeading());
        //telemetry.addData("Continuous Heading", odometerNavSource.calculateContinuousHeading());
        //telemetry.addData("Heading Wraparound", odometerNavSource.getHeadingWraparound());
        //telemetry.addData("Location", odometerNavSource.getLocation());
        telemetry.addData("Current Position", odometerNavSource.getLocation());
        telemetry.addData("Target Position", navigator.getTargetPosition());
        telemetry.addData("Current Heading", odometerNavSource.getHeading());
        telemetry.addData("Target Heading", navigator.getTargetHeading());
        telemetry.addData("At Target", navigator.atTarget());
        telemetry.addData("At Positional Target", pidfMovementStrategy.atPositionalTarget());
        telemetry.addData("At Heading Target", pidfMovementStrategy.atHeadingTarget());
    }

    @Override
    public void stop() {
        navigator.setEnabled(false);
    }
}
