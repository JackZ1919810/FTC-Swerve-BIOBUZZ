package org.firstinspires.ftc.teamcode.util;

/** Which of our two HIVE CELLS faces up. Only the driver knows, so it starts UNKNOWN every TeleOp. */
public enum UpCell {
    UNKNOWN, AUDIENCE, REAR;

    /**
     * The CELL on the driver's right or left. A red driver faces +Y, so the audience (+X) is on
     * their right; a blue driver faces -Y, so the audience is on their left.
     */
    public static UpCell onDriversSide(boolean red, boolean right) {
        return right == red ? AUDIENCE : REAR;
    }
}
