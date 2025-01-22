package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "First_RoadRunner_Auto_2", preselectTeleOp = "Teleop_V4_Dual_Control_Main")
public class First_RoadRunner_Auto_2 extends LinearOpMode {

    // Variables used for the Arm positions
    int ARM_TICKS_PER_DEGREE = 28;
    int ARM_COLLAPSED_INTO_ROBOT = 15 * ARM_TICKS_PER_DEGREE;
    int ARM_COLLECT = 15 * ARM_TICKS_PER_DEGREE;
    int ARM_CLEAR_BARRIER = 15 * ARM_TICKS_PER_DEGREE;
    int ARM_SCORE_SPECIMEN = 50 * ARM_TICKS_PER_DEGREE;
    int ARM_ATTACH_HANGING_HOOK = 89 * ARM_TICKS_PER_DEGREE;
    int ARM_WINCH_ROBOT = 5 * ARM_TICKS_PER_DEGREE;
    int ARM_SCORE_SAMPLE_IN_LOW = 50 * ARM_TICKS_PER_DEGREE;
    int ARM_SCORE_IN_HIGH_BASKET = 65 * ARM_TICKS_PER_DEGREE;

    // Variables to store the lengths of viper slide positions.
    int SLIDE_MIN_EXTEND = 0;
    int SLIDE_MAX_EXTEND = 5000;
    int SLIDE_COLLECT = 1700;
    int SLIDE_SCORE_LOW = 800;

    // Variables to store the speed the intake servo should be set at to intake, and deposit game elements.
    double CLAW_CLOSED = 0.12;
    double CLAW_OPEN_WIDE = 0.4;
    double CLAW_OPEN_SMALL = 0.3;

    // Variables to store the positions that the wrist should be set to when folding in, or folding out.
    double WRIST_FOLDED_IN = 0.25;
    double WRIST_BASKET = 0.9;
    double WRIST_COLLECT = 0.7;
    double WRIST_SPECIMEN = 0.9;
    double WRIST_FUDGE_FACTOR = 0.2;

    // A number in degrees that the triggers can adjust the arm position by
    int FUDGE_FACTOR = 10 * ARM_TICKS_PER_DEGREE;
    int SLIDE_FUDGE_FACTOR = 600;

    // Variables that are used to set Odometry Pod Servos
    double POD_UP = 1;
    double POD_DOWN = 0.4;

    //Motor and Servo hardware maps
    DcMotor left_arm;
    DcMotor viper_slide;
    Servo x_pod_lift;
    Servo y_pod_lift;
    //Servo claw;
    Servo wrist;

    public class Claw {
        //Define the Servo
        private Servo claw;
        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "intake");
        }
        //Class to Close the Claw
        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(CLAW_CLOSED);
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
                return false;
            }
        }
        public Action openSmallClaw() {
            return new OpenSmallClaw();
        }
    }

    /**
     * Sample Autonomous opMode using Roadrunner with GoBilda Pinpoint Odometry
     */
    @Override
    public void runOpMode() throws InterruptedException {
        //Instantiate Pinpoint Roadrunner drive and pose
        //Pose2d initialPose = new Pose2d(0, 0, 0);
        Pose2d initialPose = new Pose2d(23.95, -71.71, Math.toRadians(90.00));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);

        x_pod_lift = hardwareMap.get(Servo.class, "x_pod_lift");
        y_pod_lift = hardwareMap.get(Servo.class, "y_pod_lift");
        //claw = hardwareMap.get(Servo.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");
        left_arm = hardwareMap.get(DcMotor.class, "left_arm");
        viper_slide = hardwareMap.get(DcMotor.class, "viper_slide");

        // A function to set the default settings for the arm and viber slide motors
        SetMotorSettings();

        // Make sure that the intake is off, and the wrist is folded in.
        x_pod_lift.setPosition(POD_DOWN);
        y_pod_lift.setPosition(POD_DOWN);
        setTelemetry();

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(36.00, -24.00), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(48.00, -0.00), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(48.00, -60.00), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(48.00, -24.00), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(58.00, -0.00), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(58.00, -60.00), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(58.00, -24.00), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(66.00, -0.00), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(66.00, -60.00), Math.toRadians(90.00));
        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(9.83, -31.67))
                .build();

        // Actions that have to happen on Init.
        Actions.runBlocking(claw.closeClaw());



        //Waits for the start button to be pressed
        waitForStart();
        if (opModeIsActive()) {
            //Automation Run Code
            wrist.setPosition(WRIST_FOLDED_IN);
            setArmToTarget(ARM_SCORE_SPECIMEN + 8 * ARM_TICKS_PER_DEGREE);
            setViperSlideToTarget(SLIDE_SCORE_LOW + 30);

            //Move to Sumbersible
            Actions.runBlocking((
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .setTangent(0)
                            .splineToConstantHeading(new Vector2d(36, 32), Math.PI / 2)
                            .waitSeconds(1)
                            .build()
            ));


            setArmToTarget(ARM_SCORE_SPECIMEN + 0);

            //Move Back
            Actions.runBlocking((
                    drive.actionBuilder(new Pose2d(36, 32, 0))
                            .waitSeconds(1)
                            .lineToX(30)
                            .build()
            ));

            Actions.runBlocking(claw.closeClaw());
            setViperSlideToTarget(SLIDE_MIN_EXTEND);
            setArmToTarget(ARM_COLLAPSED_INTO_ROBOT);

            //spline to push
            Actions.runBlocking((
                    drive.actionBuilder(new Pose2d(30, 32, 0))
                            .strafeTo(new Vector2d(30, 0))
                            .setTangent(0)
                            .splineToSplineHeading(new Pose2d(70, -15, Math.toRadians(180)), Math.PI / 2)
                            .build()
            ));

        }


    }


    // Sets all the motor settings at once
    private void SetMotorSettings() {
        //Drive motors are set in the MecanumDrive class in Roadrunner
        //Set Odometry Servo Direction
        x_pod_lift.setDirection(Servo.Direction.FORWARD);
        y_pod_lift.setDirection(Servo.Direction.REVERSE);
        //Set Arm and Slide motor direction
        left_arm.setDirection(DcMotor.Direction.FORWARD);
        viper_slide.setDirection(DcMotor.Direction.REVERSE);
        // Reset Encoder Positions of Arm and Viper Slide to zero
        left_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viper_slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Setting zeroPowerBehavior to BRAKE enables a "brake mode".
        left_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viper_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //Moves the Arm to a target position
    private void setArmToTarget(int _position) {
        // Set the target position of the arm based on the last variable selected by the driver
        left_arm.setTargetPosition(_position);
        left_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Here we set the target velocity (speed) the motor runs at. This is in ticks-per-second.
        // This is a little under the maximum of 2800.
        ((DcMotorEx) left_arm).setVelocity(2000);
        while (opModeIsActive() && left_arm.isBusy()) {
            idle();
        }
        telemetry.addData("armPosition", _position);
    }

    // Moves Viper Slide motor to a target position
    private void setViperSlideToTarget(int _position) {
        // Set the target position of the VIPER SLIDE  based on the last variable selected by the driver
        viper_slide.setTargetPosition(_position);
        viper_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Here we set the target velocity (speed) the motor runs at. This is in ticks-per-second.
        // This is a little under the maximum of 2800.
        ((DcMotorEx) viper_slide).setVelocity(2000);
        while (opModeIsActive() && viper_slide.isBusy()) {
            idle();
        }
        telemetry.addData("slidePosition", _position);
    }

    // Updates and reports Telemetry to the driver station
    private void setTelemetry() {
        telemetry.addData("Viper Slide Position", viper_slide.getCurrentPosition());
        telemetry.addData("Viper Slide Power", viper_slide.getPower());
        telemetry.addData("Viper Slide Target", viper_slide.getTargetPosition());
        telemetry.addData("Arm Current Position:", left_arm.getCurrentPosition());
        telemetry.addData("Arm Current Power", left_arm.getPower());
        telemetry.addData("Arm Target", left_arm.getTargetPosition());
        //telemetry.addData("Claw Position", claw.getPosition());
        telemetry.addData("Wrist Position", wrist.getPosition());
        telemetry.update();
    }


}
