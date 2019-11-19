package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.control.navigation.VuMarkNavSource;

@TeleOp(name = "Vuforia Debugger")
public class VuforiaDebugger extends OpMode {

    private VuMarkNavSource vuforia;

    @Override
    public void init() {
        OpenGLMatrix phoneLocation = new OpenGLMatrix();
        phoneLocation.rotate(AxesReference.EXTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES, -90, 90, 0);
        vuforia = new VuMarkNavSource(hardwareMap.appContext, VuforiaKey.KEY, VuforiaLocalizer.CameraDirection.BACK, phoneLocation);
    }

    @Override
    public void loop() {
        telemetry.addData("Can See Stone", vuforia.canSeeStone());
        OpenGLMatrix stoneLocation = vuforia.getRelativeStoneLocation();
        if (stoneLocation != null) {
            VectorF translation = stoneLocation.getTranslation();
            telemetry.addData(
                    "Translation",
                    "%f, %f, %f",
                    translation.get(0),
                    translation.get(1),
                    translation.get(2)
            );
        }
    }
}
