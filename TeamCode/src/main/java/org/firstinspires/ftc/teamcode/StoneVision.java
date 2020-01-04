package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.Rect;
import android.os.SystemClock;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Frame;
import com.vuforia.RectangleInt;

import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.ftc9974.thorcore.util.MathUtilities;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.IntBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.stream.Stream;

public class StoneVision {

    public static class Tunings {

        public static Tunings RED = new Tunings(
                150, 380, 300, 215,
                429, 380, 300, 215,
                708, 380, 300, 215);
        public static Tunings BLUE = new Tunings(
                217, 389, 313, 190,
                492, 389, 313, 190,
                773, 389, 313, 190
        );

        public int leftX, leftY, leftWidth, leftHeight;
        public int centerX, centerY, centerWidth, centerHeight;
        public int rightX, rightY, rightWidth, rightHeight;

        public Tunings(int leftX, int leftY, int leftWidth, int leftHeight,
                       int centerX, int centerY, int centerWidth, int centerHeight,
                       int rightX, int rightY, int rightWidth, int rightHeight) {
            this.leftX = leftX;
            this.leftY = leftY;
            this.leftWidth = leftWidth;
            this.leftHeight = leftHeight;
            this.centerX = centerX;
            this.centerY = centerY;
            this.centerWidth = centerWidth;
            this.centerHeight = centerHeight;
            this.rightX = rightX;
            this.rightY = rightY;
            this.rightWidth = rightWidth;
            this.rightHeight = rightHeight;
        }
    }

    private Tunings tunings;

    private VuforiaLocalizer vuforia;

    private AtomicBoolean processingComplete;

    private volatile StonePosition position;
    public volatile double time;

    private ElapsedTime timer;

    public StoneVision(VuforiaLocalizer vuforia, Tunings tunings) {
        this.vuforia = vuforia;
        this.vuforia.enableConvertFrameToBitmap();

        this.tunings = tunings;
        timer = new ElapsedTime();

        processingComplete = new AtomicBoolean(false);
    }

    public void beginProcessing() {
        timer.reset();
        vuforia.getFrameOnce(Continuation.createTrivial(this::consumeFrame));
    }

    public boolean isProcessingComplete() {
        return processingComplete.get();
    }

    public StonePosition getStonePosition() {
        if (!isProcessingComplete()) {
            return StonePosition.UNKNOWN;
        }
        return position;
    }

    private void consumeFrame(Frame frame) {
        // (0, 0) is top right when phone is held in portrait mode
        // x=1280
        // y=720
        // (0,0) -------- 1279
        //   |
        //   |
        //   |
        //   |
        //  719
        Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
        if (bitmap == null) {
            throw new IllegalArgumentException("Got a null Bitmap!");
        }

        //saveBitmap(bitmap, "Frame");

        double leftMatchFactor = processRegion(bitmap, tunings.leftX, tunings.leftY, tunings.leftWidth, tunings.leftHeight);
        double centerMatchFactor = processRegion(bitmap, tunings.centerX, tunings.centerY, tunings.centerWidth, tunings.centerHeight);
        double rightMatchFactor = processRegion(bitmap, tunings.rightX, tunings.rightY, tunings.rightWidth, tunings.rightHeight);

        double min = MathUtilities.min(leftMatchFactor, centerMatchFactor, rightMatchFactor);

        if (min == leftMatchFactor) {
            position = StonePosition.LEFT;
        } else if (min == centerMatchFactor) {
            position = StonePosition.CENTER;
        } else {
            position = StonePosition.RIGHT;
        }

        processingComplete.set(true);
        time = timer.seconds();
    }

    private void saveBitmap(Bitmap bmp, String name) {
        try (FileOutputStream fos = new FileOutputStream(new File(AppUtil.FIRST_FOLDER + "/" + name + ".png"))) {
            bmp.compress(Bitmap.CompressFormat.PNG, 100, fos);
        } catch (FileNotFoundException e) {
            RobotLog.ee("StoneVision", e, "File Not Found");
        } catch (IOException e) {
            RobotLog.ee("StoneVision", e, "IOException while writing file");
        }
    }

    private double processRegion(Bitmap bitmap, int x, int y, int width, int height) {
        // TODO: 12/26/19 Implement this using RenderScript
        int[] pixels = new int[width * height];
        bitmap.getPixels(pixels, 0, width, x, y, width, height);

        /*Bitmap original = Bitmap.createBitmap(bitmap, x, y, width, height);

        Bitmap hue = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888),
                saturation = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888),
                value = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888),
                matchFactor = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        int[] huePixels = Arrays.stream(pixels)
                .parallel()
                .map((color) -> {
                    float[] hsv = new float[3];
                    Color.colorToHSV(color, hsv);
                    hsv[1] = hsv[2] = 1;
                    return Color.HSVToColor(hsv);
                }).toArray();
        int[] saturationPixels = Arrays.stream(pixels)
                .parallel()
                .map((color) -> {
                    float[] hsv = new float[3];
                    Color.colorToHSV(color, hsv);
                    hsv[2] = hsv[1];
                    hsv[1] = 0;
                    return Color.HSVToColor(hsv);
                }).toArray();
        int[] valuePixels = Arrays.stream(pixels)
                .parallel()
                .map((color) -> {
                    float[] hsv = new float[3];
                    Color.colorToHSV(color, hsv);
                    hsv[1] = 0;
                    return Color.HSVToColor(hsv);
                }).toArray();
        int[] matchFactorPixels = Arrays.stream(pixels)
                .parallel()
                .map((color) -> {
                    return Color.HSVToColor(new float[] {0, 0, (float) calculateMatchFactor(color)});
                }).toArray();

        hue.setPixels(huePixels, 0, width, 0, 0, width, height);
        saturation.setPixels(saturationPixels, 0, width, 0, 0, width, height);
        value.setPixels(valuePixels, 0, width, 0, 0, width, height);
        matchFactor.setPixels(matchFactorPixels, 0, width, 0, 0, width, height);

        saveBitmap(original, String.format("%d-ORG", x));
        saveBitmap(hue, String.format("%d-HUE", x));
        saveBitmap(saturation, String.format("%d-SAT", x));
        saveBitmap(value, String.format("%d-VAL", x));
        saveBitmap(matchFactor, String.format("%d-MF", x));

        original.recycle();
        hue.recycle();
        saturation.recycle();
        value.recycle();
        matchFactor.recycle();*/

        return Arrays.stream(pixels)
                .parallel()
                .mapToDouble(this::calculateMatchFactor)
                .sum();
    }

    private double calculateMatchFactor(int color) {
        float[] hsv = new float[3];

        Color.colorToHSV(color, hsv);

        return hsv[2];
    }
}
