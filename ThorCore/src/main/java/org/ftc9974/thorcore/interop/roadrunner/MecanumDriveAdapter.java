package org.ftc9974.thorcore.interop.roadrunner;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.ftc9974.thorcore.control.navigation.NavSource;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;

public final class MecanumDriveAdapter extends com.acmerobotics.roadrunner.drive.MecanumDrive {

    private org.ftc9974.thorcore.robot.drivetrains.MecanumDrive rb;
    private NavSource headingSource;
    private double tpmm;
    public double lfl, lbl, lbr, lfr;
    public boolean hasBeenUsed;

    public MecanumDriveAdapter(org.ftc9974.thorcore.robot.drivetrains.MecanumDrive rb, NavSource headingSource, double trackWidth, double wheelbase, double ticksPerMillimeter) {
        super(trackWidth, wheelbase);
        this.rb = rb;
        this.headingSource = headingSource;
        tpmm = ticksPerMillimeter;
        rb.setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public double getExternalHeading() {
        return headingSource.getHeading();
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        int[] posArray = rb.getEncoderPositions();
        return Arrays.asList(posArray[0] / tpmm, posArray[2] / tpmm, posArray[3] / tpmm, posArray[1] / tpmm);
    }

    @Override
    public void setMotorPowers(double frontLeft, double backLeft, double backRight, double frontRight) {
        hasBeenUsed = true;
        lfl = frontLeft;
        lbl = backLeft;
        lbr = backRight;
        lfr = frontRight;
        rb.setMotorPowers(new double[] {frontLeft, frontRight, backLeft, backRight});
    }
}
