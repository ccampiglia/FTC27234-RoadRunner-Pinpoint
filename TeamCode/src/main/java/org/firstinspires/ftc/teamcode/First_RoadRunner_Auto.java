package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "First_RoadRunner_Auto", preselectTeleOp = "Teleop_V4_Dual_Control_Main")
public class First_RoadRunner_Auto extends LinearOpMode {

    // Variables used for the Arm positions
    double ARM_TICKS_PER_DEGREE = 28;
    double ARM_COLLAPSED_INTO_ROBOT = 15 * ARM_TICKS_PER_DEGREE;
    double ARM_COLLECT = 10 * ARM_TICKS_PER_DEGREE;
    double ARM_CLEAR_BARRIER = 15 * ARM_TICKS_PER_DEGREE;
    double ARM_SCORE_SPECIMEN = 45 * ARM_TICKS_PER_DEGREE;
    double ARM_ATTACH_HANGING_HOOK = 95 * ARM_TICKS_PER_DEGREE;
    double ARM_WINCH_ROBOT = 10 * ARM_TICKS_PER_DEGREE;
    double ARM_SCORE_SAMPLE_IN_LOW = 50 * ARM_TICKS_PER_DEGREE;
    double ARM_SCORE_IN_HIGH_BASKEST = 65 * ARM_TICKS_PER_DEGREE;

    // Variables to store the lengths of viper slide positions.
    double SLIDE_MIN_EXTEND = 0;
    double SLIDE_MAX_EXTEND = 4000;
    double SLIDE_COLLECT = 1500;
    double SLIDE_SCORE_LOW = 863;

    // Variables to store the speed the intake servo should be set at to intake, and deposit game elements.
    double INTAKE_HOLD_IT = -0.3;
    double INTAKE_COLLECT = -1;
    double INTAKE_OFF = 0;
    double INTAKE_DEPOSIT = 0.3;

    // Variables to store the positions that the wrist should be set to when folding in, or folding out.
    double WRIST_FOLDED_IN = 0.1;
    double WRIST_FOLDED_OUT = 0.8;
    double WRIST_CENTERED = 0.49;

    // A number in degrees that the triggers can adjust the arm position by
    double FUDGE_FACTOR = 10 * ARM_TICKS_PER_DEGREE;
    double SLIDE_FUDGE_FACTOR = 600;

    // Variables that are used to set Odometry Pod Servos
    double POD_UP = 1;
    double POD_DOWN = 0.4;

    //Motor and Servo hardware maps
    DcMotor left_arm;
    DcMotor viper_slide;
    Servo x_pod_lift;
    Servo y_pod_lift;
    CRServo intake;
    Servo wrist;

    /**
     * Sample Autonomous opMode using Roadrunner with GoBilda Pinpoint Odometry
     */
    @Override
    public void runOpMode() throws InterruptedException {
        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));

        x_pod_lift = hardwareMap.get(Servo.class, "x_pod_lift");
        y_pod_lift = hardwareMap.get(Servo.class, "y_pod_lift");
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");
        left_arm = hardwareMap.get(DcMotor.class, "left_arm");
        viper_slide = hardwareMap.get(DcMotor.class, "viper_slide");


        // A function to set the default settings for the arm and viber slide motors
        SetMotorSettings();

        // Make sure that the intake is off, and the wrist is folded in.
        x_pod_lift.setPosition(POD_DOWN);
        y_pod_lift.setPosition(POD_DOWN);
        intake.setPower(0);
        setTelemetry();

        //Waits for the start button to be pressed
        waitForStart();
        if (opModeIsActive()) {
            intake.setPower(INTAKE_HOLD_IT);
            wrist.setPosition(WRIST_CENTERED);
            // Put run blocks here.
            setArmToTarget(ARM_SCORE_SPECIMEN + 8 * ARM_TICKS_PER_DEGREE);
            setViperSlideToTarget(SLIDE_SCORE_LOW + 30);

            //Drive forward 10 inches
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .lineToX(10)
                            .lineToY(6)
                            .lineToX(15)
                            .build());

            setArmToTarget(ARM_SCORE_SPECIMEN + 0);
            sleep(200);


            drive("FORWARD", 100, 100, 0.3);
            drive("LEFT", 600, 600, 0.3);
            drive("FORWARD", 650, 650, 0.3);
            sleep(200);
            setArmToTarget(ARM_SCORE_SPECIMEN + 0);
            sleep(200);
            drive("REV", -650, -650, 0.3);
            drive("RIGHT", 1200, 1200, 0.4);

            intake.setPower(INTAKE_OFF);
            setViperSlideToTarget(SLIDE_MIN_EXTEND);
            setArmToTarget(ARM_COLLAPSED_INTO_ROBOT);
        }
    }

    // Sets all the motor settings at once
    private void SetMotorSettings() {
        // Most skid-steer/differential drive robots require reversing one motor to drive forward.
        //Drive motors are set in the MecanumDrive class in Roadrunner
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
        // Set the target postiion of the arm based on the last variable selected by the driver
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
        // Set the target postiion of the VIPER SLIDE  based on the last variable selected by the driver
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
        telemetry.addData("Encoder Position", left_front_drive.getCurrentPosition());
        telemetry.update();
    }

    public class ServoAction implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }
}
