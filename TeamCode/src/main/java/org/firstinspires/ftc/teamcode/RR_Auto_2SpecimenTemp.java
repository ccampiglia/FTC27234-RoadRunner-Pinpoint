package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.AutoVariables.ARM_ATTACH_SPECIMEN;
import static org.firstinspires.ftc.teamcode.AutoVariables.ARM_COLLAPSED_INTO_ROBOT;
import static org.firstinspires.ftc.teamcode.AutoVariables.ARM_COLLECT_SPECIMEN;
import static org.firstinspires.ftc.teamcode.AutoVariables.ARM_SCORE_SPECIMEN;
import static org.firstinspires.ftc.teamcode.AutoVariables.POD_DOWN;
import static org.firstinspires.ftc.teamcode.AutoVariables.SLIDE_COLLECT;
import static org.firstinspires.ftc.teamcode.AutoVariables.SLIDE_MIN_EXTEND;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "RR_Auto_2SpecimenTemp", preselectTeleOp = "Teleop_V4_Dual_Control_Main")
public class RR_Auto_2SpecimenTemp extends LinearOpMode {
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
        ClawMode claw = new ClawMode(hardwareMap);
        WristMode wrist = new WristMode(hardwareMap);
        ArmMode arm = new ArmMode(hardwareMap);
        SlideMode slide = new SlideMode(hardwareMap);

        ///Set override for max velocity constraints
        double velocityOverride = 1000.0;

        /// A function to set the default settings for the arm and viber slide motors
        SetMotorSettings();

        ///**** Move forward and spline behind first block and push back to OZ
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(40.0, -40.0, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(15.0)
                .strafeToLinearHeading(new Vector2d(50, 15), Math.toRadians(-90.00), new TranslationalVelConstraint(velocityOverride));
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToY(-46);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .lineToY(-40);
        TrajectoryActionBuilder tab4 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(0, -35), Math.toRadians(90.00), new TranslationalVelConstraint(velocityOverride));
        TrajectoryActionBuilder tab5 = drive.actionBuilder(initialPose)
                .lineToY(-40);
        TrajectoryActionBuilder tab6 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(52, -45), Math.toRadians(-90.00), new TranslationalVelConstraint(velocityOverride));


        Action trajectoryStartSequence = tab1.build();
        Action trajectoryPushBlock = tab2.endTrajectory().fresh().build();
        Action trajectoryBackupFromOZ = tab3.endTrajectory().fresh().build();
        Action trajectoryMoveToSubmersible1 = tab4.endTrajectory().fresh().build();
        Action trajectoryBackupFromSubmersible1 = tab5.endTrajectory().fresh().build();
        Action trajectoryStrafeToOZ = tab6.endTrajectory().fresh().build();


        /// Initialize positions.
        xPodLiftServo.setPosition(POD_DOWN);
        yPodLiftServo.setPosition(POD_DOWN);
        setTelemetry();


        ///Waits for the start button to be pressed
        waitForStart();
        if (opModeIsActive()) {


            ///Automation Run Code
            Actions.runBlocking(
                    drive.actionBuilder(initialPose)
                            ///**** Move forward and spline behind first block and push back to OZ
                            .splineToLinearHeading(new Pose2d(40.0, -40.0, Math.toRadians(90)), Math.toRadians(90))
                            .lineToY(15.0)
                            //.splineTo(new Vector2d(48.00, 15.00), Math.toRadians(-90))
                            .strafeToLinearHeading(new Vector2d(50, 15), Math.toRadians(-90.00), new TranslationalVelConstraint(velocityOverride))

                            .stopAndAdd(arm.moveArmToPosition(ARM_COLLECT_SPECIMEN))
                            .stopAndAdd(slide.moveSlideToPosition(SLIDE_MIN_EXTEND))
                            .stopAndAdd(wrist.wristCollect())
                            .stopAndAdd(claw.openWideClaw())
                            .lineToY(-46)

                            ///GrabSpecimen
                            .stopAndAdd(claw.closeClaw())
                            .stopAndAdd(arm.moveArmToPosition(ARM_SCORE_SPECIMEN))

                            ///Back up and Move to Submersible
                            .lineToY(-40)
                            .stopAndAdd(wrist.wristScoreSpecimen())
                            .strafeToLinearHeading(new Vector2d(-5, -36), Math.toRadians(90.00), new TranslationalVelConstraint(velocityOverride))
                            .stopAndAdd(slide.moveSlideToPosition(SLIDE_COLLECT))

                            //.setTangent(Math.toRadians(90.00))
                            //.lineToY(-34.00)

                            ///Score Specimen
                            .waitSeconds(1)
                            .stopAndAdd(arm.moveArmToPosition(ARM_ATTACH_SPECIMEN))
                            .stopAndAdd(wrist.wristHangSpecimen())
                            .stopAndAdd(claw.openSmallClaw())
                            .stopAndAdd(slide.moveSlideToPosition(SLIDE_MIN_EXTEND))
                            .lineToY(-40, new TranslationalVelConstraint(velocityOverride))


                            ///Strafe Back for 2nd Specimen
                            .strafeToLinearHeading(new Vector2d(52, -45), Math.toRadians(-90.00), new TranslationalVelConstraint(velocityOverride))
                            .stopAndAdd(arm.moveArmToPosition(ARM_COLLECT_SPECIMEN))
                            .stopAndAdd(wrist.wristCollect())

                            ///GrabSpecimen
                            .stopAndAdd(claw.closeClaw())

                            ///Move to Submersible
                            .stopAndAdd(arm.moveArmToPosition(ARM_SCORE_SPECIMEN))
                            .lineToY(-40)
                            .stopAndAdd(wrist.wristScoreSpecimen())
                            .strafeToLinearHeading(new Vector2d(-10, -36), Math.toRadians(90.00), new TranslationalVelConstraint(velocityOverride))
                            .stopAndAdd(slide.moveSlideToPosition(SLIDE_COLLECT))
                            //.setTangent(Math.toRadians(90.00))
                            //.lineToY(-34.00)

                            ///Score Specimen
                            .waitSeconds(1)
                            .stopAndAdd(arm.moveArmToPosition(ARM_ATTACH_SPECIMEN))
                            .stopAndAdd(wrist.wristHangSpecimen())
                            .stopAndAdd(claw.openSmallClaw())
                            .stopAndAdd(slide.moveSlideToPosition(SLIDE_MIN_EXTEND))
                            .lineToY(-40, new TranslationalVelConstraint(velocityOverride))

                            ///Move back to OZ
                            .strafeToLinearHeading(new Vector2d(57, -57), Math.toRadians(90.00), new TranslationalVelConstraint(velocityOverride))
                            .stopAndAdd(wrist.wristFoldedIn())
                            .stopAndAdd(claw.closeClaw())
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
