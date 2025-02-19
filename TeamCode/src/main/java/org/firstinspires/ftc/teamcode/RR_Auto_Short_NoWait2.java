package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.AutoVariables.ARM_ATTACH_SPECIMEN;
import static org.firstinspires.ftc.teamcode.AutoVariables.ARM_COLLAPSED_INTO_ROBOT;
import static org.firstinspires.ftc.teamcode.AutoVariables.ARM_COLLECT_SPECIMEN;
import static org.firstinspires.ftc.teamcode.AutoVariables.ARM_SCORE_SPECIMEN;
import static org.firstinspires.ftc.teamcode.AutoVariables.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.AutoVariables.CLAW_OPEN_SMALL;
import static org.firstinspires.ftc.teamcode.AutoVariables.CLAW_OPEN_WIDE;
import static org.firstinspires.ftc.teamcode.AutoVariables.POD_DOWN;
import static org.firstinspires.ftc.teamcode.AutoVariables.SLIDE_MIN_EXTEND;
import static org.firstinspires.ftc.teamcode.AutoVariables.SLIDE_SCORE_SPECIMEN;
import static org.firstinspires.ftc.teamcode.AutoVariables.WRIST_COLLECT_SPECIMEN;
import static org.firstinspires.ftc.teamcode.AutoVariables.WRIST_COLLECT_SPECIMEN2;
import static org.firstinspires.ftc.teamcode.AutoVariables.WRIST_FOLDED_IN;
import static org.firstinspires.ftc.teamcode.AutoVariables.WRIST_HANG_SPECIMEN;
import static org.firstinspires.ftc.teamcode.AutoVariables.WRIST_SCORE_SPECIMEN;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "RR_Auto_Short_NoWait2", preselectTeleOp = "TeleopClaw_V5_Dual_Control_MAIN")
public class RR_Auto_Short_NoWait2 extends LinearOpMode {

    /// Motor and Servo hardware maps
    DcMotor left_arm;
    DcMotor viper_slide;
    Servo clawServo;
    Servo wristServo;
    Servo xPodLiftServo;
    Servo yPodLiftServo;

    /**
     * Sample Autonomous opMode using Roadrunner with GoBilda Pinpoint Odometry
     */
    @Override
    public void runOpMode() throws InterruptedException {
        ///Initialize Motors and Servos
        left_arm = hardwareMap.get(DcMotor.class, "left_arm");
        viper_slide = hardwareMap.get(DcMotor.class, "viper_slide");
        clawServo = hardwareMap.get(Servo.class, "intake");
        wristServo = hardwareMap.get(Servo.class, "wrist");
        xPodLiftServo = hardwareMap.get(Servo.class, "x_pod_lift");
        yPodLiftServo = hardwareMap.get(Servo.class, "y_pod_lift");

        ///Instantiate Pinpoint Roadrunner drive and pose
        //Pose2d initialPose = new Pose2d(0, 0, 0);
        Pose2d initialPose = new Pose2d(24, -63, Math.toRadians(90.00));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Slide slide = new Slide(hardwareMap);

        /// A function to set the default settings for the arm and viber slide motors
        SetMotorSettings();

        /// Initialize positions.
        claw.closeClaw();
        wrist.wristFoldedIn();
        arm.moveArmToPosition(ARM_COLLAPSED_INTO_ROBOT);
        xPodLiftServo.setPosition(POD_DOWN);
        yPodLiftServo.setPosition(POD_DOWN);
        setTelemetry();

        double velocityOverride = 100.0;

        ///Waits for the start button to be pressed
        waitForStart();
        if (opModeIsActive()) {
            ///Automation Run Code
            ///Push Blocks
            Actions.runBlocking(
                    drive.actionBuilder(initialPose)
                            ///**** Move forward and spline behind first block and push back to OZ
                            //.splineToLinearHeading(new Pose2d(40.0, -40.0, Math.toRadians(90)), Math.toRadians(90))
                            //.lineToY(15.0)
                            //.strafeToLinearHeading(new Vector2d(53, 15), Math.toRadians(-90.00), new TranslationalVelConstraint(velocityOverride))

                            ///Set up for Collection
                            .stopAndAdd(arm.moveArmToPosition(ARM_COLLECT_SPECIMEN))
                            .afterTime(1,slide.moveSlideToPosition(SLIDE_MIN_EXTEND))
                            .afterTime(2,wrist.wristCollect())
                            .afterTime(2,claw.openWideClaw())

                            ///Go to Observation Zone to collect first specimen
                            .strafeToLinearHeading(new Vector2d(52, -35), Math.toRadians(-90.00), new TranslationalVelConstraint(velocityOverride))
                            .lineToY(-45)

                            ///Grab First Specimen
                            .waitSeconds(1)
                            .stopAndAdd(claw.closeClaw())
                            .stopAndAdd(arm.moveArmToPosition(ARM_SCORE_SPECIMEN))

                            ///Back up and Move to Submersible
                            //.lineToY(-40)
                            .stopAndAdd(wrist.wristScoreSpecimen())
                            .strafeToLinearHeading(new Vector2d(-5, -33.0), Math.toRadians(90.00), new TranslationalVelConstraint(velocityOverride))

                            //.setTangent(Math.toRadians(90.00))
                            //.lineToY(-34.00)

                            ///Score First Specimen
                            .stopAndAdd(slide.moveSlideToPosition(SLIDE_SCORE_SPECIMEN))
                            .waitSeconds(.5)
                            .stopAndAdd(arm.moveArmToPosition(ARM_ATTACH_SPECIMEN))
                            .stopAndAdd(wrist.wristHangSpecimen())
                            .stopAndAdd(claw.openSmallClaw())
                            .stopAndAdd(slide.moveSlideToPosition(SLIDE_MIN_EXTEND))
                            //.lineToY(-40, new TranslationalVelConstraint(velocityOverride))


                            ///Strafe Back for 2nd Specimen
                            .strafeToLinearHeading(new Vector2d(52, -44.5), Math.toRadians(-90.00), new TranslationalVelConstraint(velocityOverride))

                            ///Grab Second Specimen
                            .stopAndAdd(arm.moveArmToPosition(ARM_COLLECT_SPECIMEN))
                            .stopAndAdd(wrist.wristCollect())
                            .waitSeconds(1.5)
                            .stopAndAdd(claw.closeClaw())
                            .stopAndAdd(arm.moveArmToPosition(ARM_SCORE_SPECIMEN))

                            ///Move to Submersible
                            .stopAndAdd(wrist.wristScoreSpecimen())
                            .strafeToLinearHeading(new Vector2d(-15, -33.0), Math.toRadians(90.00), new TranslationalVelConstraint(velocityOverride))

                            ///Score Second Specimen
                            .stopAndAdd(slide.moveSlideToPosition(SLIDE_SCORE_SPECIMEN))
                            .waitSeconds(.5)
                            .stopAndAdd(arm.moveArmToPosition(ARM_ATTACH_SPECIMEN))
                            .stopAndAdd(wrist.wristHangSpecimen())
                            //.stopAndAdd(wrist.wristFoldedIn())
                            .stopAndAdd(claw.openSmallClaw())
                            .stopAndAdd(slide.moveSlideToPosition(SLIDE_MIN_EXTEND))
                            //.lineToY(-40, new TranslationalVelConstraint(velocityOverride))

                            ///Move back to OZ
                            .strafeToLinearHeading(new Vector2d(57, -57), Math.toRadians(90.00))
                            .stopAndAdd(wrist.wristFoldedIn())
                            .stopAndAdd(claw.closeClaw())
                            .stopAndAdd(arm.moveArmToPosition(ARM_COLLAPSED_INTO_ROBOT))
                            //.waitSeconds(1)
                            .build()
            );
        }
    }

    /// Sets all the motor settings at once
    private void SetMotorSettings() {
        ///Drive motors are set in the MecanumDrive class in Roadrunner
        ///Set Odometry Servo Direction
        xPodLiftServo.setDirection(Servo.Direction.FORWARD);
        yPodLiftServo.setDirection(Servo.Direction.REVERSE);
        ///Set Arm and Slide motor direction
        left_arm.setDirection(DcMotor.Direction.FORWARD);
        viper_slide.setDirection(DcMotor.Direction.REVERSE);
        /// Reset Encoder Positions of Arm and Viper Slide to zero
        left_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viper_slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /// Setting zeroPowerBehavior to BRAKE enables a "brake mode".
        left_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viper_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //************************************************************
    //**** Create Class and Methods for Wrist control
    public class Wrist {
        //Define the Servo
        private final Servo wrist;

        public Wrist(HardwareMap hardwareMap) {
            wrist = hardwareMap.get(Servo.class, "wrist");
        }

        //Class to Put Wrist in Basket Position
        public class WristFoldedIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(WRIST_FOLDED_IN);
                return false;
            }
        }

        public Action wristFoldedIn() {
            return new WristFoldedIn();
        }

        //Class to Put Wrist in Collect Position
        public class WristCollect implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(WRIST_COLLECT_SPECIMEN);
                sleep(800);
                return false;
            }
        }

        public Action wristCollect() {
            return new WristCollect();
        }

        //Class to Put Wrist in Collect Position
        public class WristCollect2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(WRIST_COLLECT_SPECIMEN2);
                sleep(800);
                return false;
            }
        }

        public Action wristCollect2() {
            return new WristCollect2();
        }

        //Class to Put Wrist in Specimen Score Position
        public class WristScoreSpecimen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(WRIST_SCORE_SPECIMEN);
                sleep(500);
                return false;
            }
        }

        public Action wristScoreSpecimen() {
            return new WristScoreSpecimen();
        }

        //Class to Put Wrist in Specimen Score Position
        public class WristHangSpecimen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(WRIST_HANG_SPECIMEN);
                sleep(300);
                return false;
            }
        }

        public Action wristHangSpecimen() {
            return new WristHangSpecimen();
        }
    }

    //************************************************************
    //**** Create Class and Methods for Claw control
    public class Claw {
        //Define the Servo
        private final Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "intake");
        }

        //Class to Close the Claw
        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(CLAW_CLOSED);
                sleep(800);
                return false;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        //Class to Open the Claw Wide
        public class OpenWideClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(CLAW_OPEN_WIDE);
                sleep(300);
                return false;
            }
        }

        public Action openWideClaw() {
            return new OpenWideClaw();
        }

        //Class to Open the Claw Narrow
        public class OpenSmallClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(CLAW_OPEN_SMALL);
                sleep(300);
                return false;
            }
        }

        public Action openSmallClaw() {
            return new OpenSmallClaw();
        }
    }

    //int ARM_COLLAPSED_INTO_ROBOT = 15 * ARM_TICKS_PER_DEGREE;
    //int ARM_COLLECT_SPECIMEN = 15 * ARM_TICKS_PER_DEGREE;
    //int ARM_SCORE_SPECIMEN = 50 * ARM_TICKS_PER_DEGREE;
    //int ARM_SCORE_IN_HIGH_BASKET = 65 * ARM_TICKS_PER_DEGREE;
    //************************************************************
    //**** Create Class Arm Control
    public class Arm {
        //Define the Motor
        private final DcMotor arm;

        public Arm(HardwareMap hardwareMap) {
            arm = hardwareMap.get(DcMotor.class, "left_arm");
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        //Class  - Move to Arm Position
        public class MoveArmToPosition implements Action {
            int armPosition;

            public MoveArmToPosition(int p) {
                this.armPosition = p;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setTargetPosition(armPosition);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ((DcMotorEx) arm).setVelocity(2800);
                while (opModeIsActive() && arm.isBusy()) {
                    idle();
                }
                telemetry.addData("armPosition", arm.getCurrentPosition());
                return false;
            }
        }

        public Action moveArmToPosition(int p) {
            return new MoveArmToPosition(p);
        }
    }

    //************************************************************
    //**** Create Class Viper Slide Control
    public class Slide {
        //Define the Motor
        private final DcMotor slide;

        public Slide(HardwareMap hardwareMap) {
            slide = hardwareMap.get(DcMotor.class, "viper_slide");
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slide.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        //Class  - Move to Arm Position
        public class MoveSlideToPosition implements Action {
            int slidePosition;

            public MoveSlideToPosition(int p) {
                this.slidePosition = p;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slide.setTargetPosition(slidePosition);
                slide.setPower(1);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(1);
                //((DcMotorEx) slide).setVelocity(2800);
                while (opModeIsActive() && slide.isBusy()) {
                    idle();
                }
                telemetry.addData("armPosition", slide.getCurrentPosition());
                return false;
            }
        }

        public Action moveSlideToPosition(int p) {
            return new MoveSlideToPosition(p);
        }
    }

    /// Updates and reports Telemetry to the driver station
    private void setTelemetry() {
        telemetry.addData("Viper Slide Position", viper_slide.getCurrentPosition());
        telemetry.addData("Viper Slide Power", viper_slide.getPower());
        telemetry.addData("Viper Slide Target", viper_slide.getTargetPosition());
        telemetry.addData("Arm Current Position:", left_arm.getCurrentPosition());
        telemetry.addData("Arm Current Power", left_arm.getPower());
        telemetry.addData("Arm Target", left_arm.getTargetPosition());
        telemetry.addData("Claw Position", clawServo.getPosition());
        telemetry.addData("Wrist Position", wristServo.getPosition());
        telemetry.update();
    }


}
