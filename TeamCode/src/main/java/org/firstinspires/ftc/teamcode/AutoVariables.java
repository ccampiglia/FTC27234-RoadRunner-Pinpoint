package org.firstinspires.ftc.teamcode;

public class AutoVariables {
    /// Variables used for the Arm positions
    public static final int ARM_TICKS_PER_DEGREE = 28;
    public static final int ARM_COLLAPSED_INTO_ROBOT = 10 * ARM_TICKS_PER_DEGREE;
    public static final int ARM_COLLECT_SPECIMEN = 18 * ARM_TICKS_PER_DEGREE;
    public static final int ARM_COLLECT_SPECIMEN2 = 22 * ARM_TICKS_PER_DEGREE;
    public static final int ARM_SCORE_SPECIMEN = 55 * ARM_TICKS_PER_DEGREE;
    public static final int ARM_ATTACH_SPECIMEN = 37 * ARM_TICKS_PER_DEGREE;
    public static final int ARM_SCORE_BASKET = 65 * ARM_TICKS_PER_DEGREE;
    public static final int ARM_COLLECT_FLOOR = 13 * ARM_TICKS_PER_DEGREE;


    /// Variables to store the lengths of viper slide positions.
    public static final int SLIDE_TICKS_PER_INCH = 112;
    public static final int SLIDE_MIN_EXTEND = 0 * SLIDE_TICKS_PER_INCH;
    public static final int SLIDE_MAX_EXTEND = 17 * SLIDE_TICKS_PER_INCH;
    public static final int SLIDE_COLLECT = 8 * SLIDE_TICKS_PER_INCH;
    public static final int SLIDE_SCORE_SPECIMEN = 4 * SLIDE_TICKS_PER_INCH;  //4
    public static final int SLIDE_SCORE_BASKET = 17 * SLIDE_TICKS_PER_INCH;  //4


    /// Variables to store the speed the intake servo should be set at to intake, and deposit game elements.
    public static final double CLAW_CLOSED = 0.140;
    public static final double CLAW_OPEN_WIDE = 0.4;
    public static final double CLAW_OPEN_SMALL = 0.3;

    /// Variables to store the positions that the wrist should be set to when folding in, or folding out.
    public static final double WRIST_FOLDED_IN = 0.00;  //0,35
    public static final double WRIST_COLLECT_SPECIMEN = 0.59;  //0.9;
    public static final double WRIST_COLLECT_SPECIMEN2 = 0.9;  //0.9;
    public static final double WRIST_SCORE_SPECIMEN = 0.50;  //0.55;
    public static final double WRIST_SCORE_SPECIMEN_INITIAL = 0.8;  //0.55;
    public static final double WRIST_HANG_SPECIMEN = 0.2;  //0.5;
    public static final double WRIST_COLLECT_FLOOR = 0.35;  //0.5;



    /// Variables that are used to set Odometry Pod Servos
    public static final double POD_DOWN = 0.4;
}
