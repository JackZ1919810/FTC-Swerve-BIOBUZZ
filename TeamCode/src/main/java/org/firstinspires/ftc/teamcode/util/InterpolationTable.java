package org.firstinspires.ftc.teamcode.util;

/**
 * Lookup table with straight lines between measured points (piecewise-linear interpolation).
 * Used for shooter RPM from distance: measure a few (distance, RPM) pairs that score,
 * and this fills in every distance between them.
 *
 *   between x1 and x2:  y = y1 + (x - x1) / (x2 - x1) * (y2 - y1)
 *   outside the table:  the closest end value (no guessing past what was measured)
 */
public final class InterpolationTable {
    private InterpolationTable() {}

    /**
     * @param xs measured inputs, smallest first (e.g. distance in inches)
     * @param ys measured outputs, same order and count (e.g. RPM)
     * @param x  the input to look up
     * @return y at x, or NaN if x is NaN or the table is unusable (see isValid)
     */
    public static double interpolate(double[] xs, double[] ys, double x) {
        if (Double.isNaN(x) || !isValid(xs, ys)) return Double.NaN;

        int last = xs.length - 1;
        if (x <= xs[0]) return ys[0];
        if (x >= xs[last]) return ys[last];

        // Find the two points x sits between, then go the same fraction of the way from y1 to y2
        for (int i = 0; i < last; i++) {
            if (x <= xs[i + 1]) {
                double t = (x - xs[i]) / (xs[i + 1] - xs[i]);   // 0 at xs[i], 1 at xs[i + 1]
                return ys[i] + t * (ys[i + 1] - ys[i]);
            }
        }
        return ys[last];   // not reached: x < xs[last] was checked above
    }

    /**
     * Usable table: at least one point, the same count of x and y, and x strictly increasing.
     * A typo like {36, 48, 40} makes it false, so the caller can fall back instead of using bad values.
     */
    public static boolean isValid(double[] xs, double[] ys) {
        if (xs == null || ys == null || xs.length == 0 || xs.length != ys.length) return false;
        for (int i = 0; i < xs.length - 1; i++) {
            if (!(xs[i + 1] > xs[i])) return false;   // also false for NaN
        }
        return true;
    }
}
