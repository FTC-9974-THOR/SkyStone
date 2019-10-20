package org.ftc9974.thorcore.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;

import java.util.concurrent.atomic.AtomicBoolean;

public final class UpdateLoopHandler implements OpModeManagerNotifier.Notifications {

    private AtomicBoolean stopRequested, enabled;
    private Runnable handler;
    private boolean startOnOpModeStart;

    private Thread looper;

    public UpdateLoopHandler(Runnable handler) {
        this.handler = handler;
        stopRequested = new AtomicBoolean(false);
        enabled = new AtomicBoolean(false);

        looper = new Thread(() -> {
            while (!stopRequested.get()) {
                if (enabled.get()) {
                    this.handler.run();
                }
            }
        });

        OpModeUtilities.registerListener(this);
    }

    public void startOnOpModeStart() {
        startOnOpModeStart = true;
    }

    public void start() {
        if (!looper.isAlive()) {
            looper.start();
        }
    }

    public void stop() {
        stopRequested.set(true);
    }

    public void setEnabled(boolean enabled) {
        this.enabled.set(enabled);
    }

    public boolean isEnabled() {
        return enabled.get();
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {

    }

    @Override
    public void onOpModePreStart(OpMode opMode) {
        if (startOnOpModeStart) {
            start();
        }
    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        stop();
        OpModeUtilities.unregisterListener(this);
    }
}
