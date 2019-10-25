package org.ftc9974.thorcore.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.openftc.revextensions2.RevExtensions2;

import java.util.concurrent.atomic.AtomicBoolean;

// I normally don't use singletons, but I have to use it to register a callback for OpMode state.
public final class RevExtensionsManager implements OpModeManagerNotifier.Notifications {

    static {
        revExtensionsInitialized = new AtomicBoolean(false);
        hasRegisteredListener = new AtomicBoolean(false);
        instance = new RevExtensionsManager();
    }

    private static AtomicBoolean revExtensionsInitialized, hasRegisteredListener;

    @Override
    public void onOpModePreInit(OpMode opMode) {
        init();
    }

    @Override
    public void onOpModePreStart(OpMode opMode) {

    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        revExtensionsInitialized.set(false);
    }

    private RevExtensionsManager() {}
    private static RevExtensionsManager instance;

    public static RevExtensionsManager getInstance() {
        return instance;
    }

    public synchronized void init() {
        if (!hasRegisteredListener.get()) {
            hasRegisteredListener.set(true);
            OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getActivity()).registerListener(this);
        }
        if (!revExtensionsInitialized.get()) {
            revExtensionsInitialized.set(true);
            RevExtensions2.init();
        }
    }
}
