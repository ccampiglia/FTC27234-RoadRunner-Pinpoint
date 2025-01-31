package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

//************************************************************
//**** Create Class Viper Slide Control
public class SlideMode {
    //Define the Motor
    private final DcMotor slide;

    public SlideMode(HardwareMap hardwareMap) {
        slide = hardwareMap.get(DcMotor.class, "viper_slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    //Class  - Move to Arm Position
    public class MoveSlideToPosition implements Action {
        int targetPosition;
        public boolean initialized = false;
        public String direction;
        public boolean returnValue = false;

        public MoveSlideToPosition(int p) {
            this.targetPosition = p;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double pos = slide.getCurrentPosition();
            packet.put("slidePos", pos);

            if (!initialized) {
                if (pos < targetPosition) {
                    slide.setPower(0.8);
                    direction = "out";
                } else {
                    slide.setPower(-0.8);
                    direction = "in";
                }
                initialized = true;
            }

            if (pos == targetPosition) {
                slide.setPower(0);
                returnValue = false;
            }

            if (direction.equals("out")) {
                if (pos < targetPosition) {
                    returnValue = true;
                } else {
                    slide.setPower(0);
                    returnValue = false;
                }
            }
            if (direction.equals("in")) {
                if (pos > targetPosition) {
                    returnValue = true;
                } else {
                    slide.setPower(0);
                    returnValue = false;
                }
            }
            return returnValue;
        }
    }


    public Action moveSlideToPosition(int p) {
        return new MoveSlideToPosition(p);
    }

    public int getPosition() {
        return slide.getCurrentPosition();
    }

}
