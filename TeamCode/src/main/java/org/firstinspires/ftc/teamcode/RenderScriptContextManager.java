package org.firstinspires.ftc.teamcode;

import android.renderscript.Allocation;
import android.renderscript.Element;
import android.renderscript.RenderScript;
import android.renderscript.Script;
import android.renderscript.Type;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

public class RenderScriptContextManager {

    private static RenderScriptContextManager instance = new RenderScriptContextManager();

    public static RenderScriptContextManager getInstance() {
        return instance;
    }

    private RenderScriptContextManager() {}

    private RenderScript rs;
    private ScriptC_GPUStoneVision gpuStoneVision;

    public void init() {
        if (rs == null) {
            rs = RenderScript.create(AppUtil.getDefContext());
            gpuStoneVision = new ScriptC_GPUStoneVision(rs);
        }
    }

    public int calculateValueSum(int[] pixels) {
        //Allocation pixelAlloc = Allocation.createSized(rs, Element.I32(rs), pixels.length);
        //pixelAlloc.copyFrom(pixels);
        /*if (gpuStoneVision == null) {
            gpuStoneVision = new ScriptC_GPUStoneVision(rs);
        }*/
        ScriptC_GPUStoneVision.result_int result = gpuStoneVision.reduce_calculateMatch(pixels);
        return result.get();
    }
}
