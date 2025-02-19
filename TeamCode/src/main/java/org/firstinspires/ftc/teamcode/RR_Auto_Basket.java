package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.AutoVariables.ARM_ATTACH_SPECIMEN;
import static org.firstinspires.ftc.teamcode.AutoVariables.ARM_COLLAPSED_INTO_ROBOT;
import static org.firstinspires.ftc.teamcode.AutoVariables.ARM_COLLECT_FLOOR;
import static org.firstinspires.ftc.teamcode.AutoVariables.ARM_COLLECT_SPECIMEN;
import static org.firstinspires.ftc.teamcode.AutoVariables.ARM_SCORE_BASKET;
import static org.firstinspires.ftc.teamcode.AutoVariables.ARM_SCORE_SPECIMEN;
import static org.firstinspires.ftc.teamcode.AutoVariables.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.AutoVariables.CLAW_OPEN_SMALL;
import static org.firstinspires.ftc.teamcode.AutoVariables.CLAW_OPEN_WIDE;
import static org.firstinspires.ftc.teamcode.AutoVariables.POD_DOWN;
import static org.firstinspires.ftc.teamcode.AutoVariables.SLIDE_COLLECT;
import static org.firstinspires.ftc.teamcode.AutoVariables.SLIDE_MAX_EXTEND;
import static org.firstinspires.ftc.teamcode.AutoVariables.SLIDE_MIN_EXTEND;
import static org.firstinspires.ftc.teamcode.AutoVariables.SLIDE_SCORE_SPECIMEN;
import static org.firstinspires.ftc.teamcode.AutoVariables.WRIST_COLLECT_FLOOR;
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

@Autonomous(name = "RR_Auto_Basket", preselectTeleOp = "TeleopClaw_V5_Dual_Control_MAIN")
public class RR_Auto_Basket extends LinearOpMode {

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
        Pose2d initialPose = new Pose2d(-24, -63, Math.toRadians(90.00));
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
            ///Set Basket Position
            int basketX = -65;
            int basketY = -41;
            int collectX1 = -60;
            int collectX2 = -70;
            int collectY = -37;
            double waitTime = 1.0;

            Actions.runBlocking(
                    drive.actionBuilder(initialPose)

                            ///Put 1st block in basket
                            .stopAndAdd(arm.moveArmToPosition(ARM_SCORE_BASKET))
                            .stopAndAdd(wrist.wristScoreSpecimen())
                            .strafeToLinearHeading(new Vector2d(basketX, basketY), Math.toRadians(225.00))
                            .stopAndAdd(slide.moveSlideToPosition(SLIDE_MAX_EXTEND))
                            .waitSeconds(waitTime)
                            .stopAndAdd(claw.openWideClaw())

                            ///Get 2nd block
                            .stopAndAdd(slide.moveSlideToPosition(SLIDE_COLLECT))
                            .strafeToLinearHeading(new Vector2d(collectX1, collectY), Math.toRadians(90))
                            .stopAndAdd(wrist.wristCollectFromFloor())
                            .stopAndAdd(arm.moveArmToPosition(ARM_COLLECT_FLOOR))
                            .waitSeconds(waitTime)
                            .stopAndAdd(claw.closeClaw())

                            ///Put 2nd block in basket
                            .stopAndAdd(arm.moveArmToPosition(ARM_SCORE_BASKET))
                            .stopAndAdd(wrist.wristScoreSpecimen())
                            .strafeToLinearHeading(new Vector2d(basketX, basketY), Math.toRadians(225.00))
                            .stopAndAdd(slide.moveSlideToPosition(SLIDE_MAX_EXTEND))
                            .waitSeconds(waitTime)
                            .stopAndAdd(claw.openWideClaw())

                            ///Get 3rd block
                            .stopAndAdd(slide.moveSlideToPosition(SLIDE_COLLECT))
                            .strafeToLinearHeading(new Vector2d(collectX2, collectY), Math.toRadians(90))
                            .stopAndAdd(wrist.wristCollectFromFloor())
                            .stopAndAdd(arm.moveArmToPosition(ARM_COLLECT_FLOOR))
                            .waitSeconds(waitTime)
                            .stopAndAdd(claw.closeClaw())

                            ///Put 3rd block in basket
                            .stopAndAdd(arm.moveArmToPosition(ARM_SCORE_BASKET))
                            .stopAndAdd(wrist.wristScoreSpecimen())
                            .strafeToLinearHeading(new Vector2d(basketX, basketY), Math.toRadians(225.00))
                            .stopAndAdd(slide.moveSlideToPosition(SLIDE_MAX_EXTEND))
                            .waitSeconds(waitTime)
                            .stopAndAdd(claw.openWideClaw())

                            /// Back to OZ
                            .waitSeconds(waitTime)
                            .stopAndAdd(slide.moveSlideToPosition(SLIDE_MIN_EXTEND))
                            .stopAndAdd(wrist.wristFoldedIn())
                            .stopAndAdd(claw.closeClaw())
                            .strafeToLinearHeading(new Vector2d(65, -57), Math.toRadians(90.00))
                            .stopAndAdd(arm.moveArmToPosition(ARM_COLLAPSED_INTO_ROBOT))
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

    ///************************************************************
    ///**** Create Class and Methods for Wrist control
    public class Wrist {
        ///Define the Servo
        private final Servo wrist;

        public Wrist(HardwareMap hardwareMap) {
            wrist = hardwareMap.get(Servo.class, "wrist");
        }

        ///Class to Put Wrist in Basket Position
        public class WristFoldedIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(WRIST_FOLDED_IN);
                return false;
            }
        }

        ///Class to Put Wrist in Collect Position
        public class WristCollect implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(WRIST_COLLECT_SPECIMEN);
                sleep(800);
                return false;
            }
        }

        ///Class to Put Wrist in Collect Position
        public class WristCollect2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(WRIST_COLLECT_SPECIMEN2);
                sleep(800);
                return false;
            }
        }

        ///Class to Put Wrist in Specimen Score Position
        public class WristScoreSpecimen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(WRIST_SCORE_SPECIMEN);
                sleep(500);
                return false;
            }
        }

        ///Class to Put Wrist in Specimen Score Position
        public class WristHangSpecimen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(WRIST_HANG_SPECIMEN);
                sleep(300);
                return false;
            }
        }

        ///Class to Put Wrist in CollectFromFloor Position
        public class WristCollectFromFloor implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(WRIST_COLLECT_FLOOR);
                sleep(300);
                return false;
            }
        }
        ///Define Actions for the classes
        public Action wristFoldedIn() {
            return new WristFoldedIn();
        }
        public Action wristCollect() {
            return new WristCollect();
        }
        public Action wristCollect2() {
            return new WristCollect2();
        }
        public Action wristScoreSpecimen() {
            return new WristScoreSpecimen();
        }
        public Action wristHangSpecimen() {
            return new WristHangSpecimen();
        }
        public Action wristCollectFromFloor() {
            return new WristCollectFromFloor();
        }
    }

    ///************************************************************
    ///**** Create Class and Methods for Claw control
    public class Claw {
        //Define the Servo
        private final Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "intake");
        }

        ///Class to Close the Claw
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

        ///Class to Open the Claw Wide
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

        ///Class to Open the Claw Narrow
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
                arm.setPower(1);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
                //((DcMotorEx) arm).setVelocity(2800);
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
