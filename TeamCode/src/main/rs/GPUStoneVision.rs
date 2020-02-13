#pragma version(1)
#pragma rs java_package_name(org.firstinspires.ftc.teamcode)
//#pragma rs_fp_full

#pragma rs reduce(calculateMatch) accumulator(calculateValue)

static void calculateValue(int *accum, int color) {
    int r = (color >> 16) & 0xff;
    int g = (color >> 8) & 0xff;
    int b = color & 0xff;

    *accum += max(max(r, g), b);
}