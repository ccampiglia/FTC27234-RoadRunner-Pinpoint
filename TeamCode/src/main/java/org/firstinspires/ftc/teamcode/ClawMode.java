package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.AutoVariables.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawMode {
    //Define the Servo
    private Servo claw;

    public ClawMode(HardwareMap hardwareMap) {
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
