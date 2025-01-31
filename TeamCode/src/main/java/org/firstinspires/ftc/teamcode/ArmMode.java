package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

//************************************************************
//**** Create Class Arm Control
public class ArmMode {
    //Define the Motor
    private final DcMotor arm;

    public ArmMode(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotor.class, "left_arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    //Class  - Move to Arm Position
    public class MoveArmToPosition implements Action {
        int targetPosition;
        public boolean initialized = false;
        public String direction;
        public boolean returnValue = false;

        public MoveArmToPosition(int p) {
            this.targetPosition = p;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double pos = arm.getCurrentPosition();
            packet.put("slidePos", pos);

            if (!initialized) {
                if (pos < targetPosition) {
                    arm.setPower(0.8);
                    direction = "up";
                } else {
                    arm.setPower(-0.8);
                    direction = "down";
                }
                initialized = true;
            }

            if (pos == targetPosition) {
                arm.setPower(0);
                returnValue = false;
            }

            if (direction.equals("up")) {
                if (pos < targetPosition) {
                    returnValue = true;
                } else {
                    arm.setPower(0);
                    returnValue = false;
                }
            }
            if (direction.equals("down")) {
                if (pos > targetPosition) {
                    returnValue = true;
                } else {
                    arm.setPower(0);
                    returnValue = false;
                }
            }
            return returnValue;
        }
    }

    public Action moveArmToPosition(int p) {
        return new MoveArmToPosition(p);
    }

    public int getPosition() {
        return arm.getCurrentPosition();
    }
}
