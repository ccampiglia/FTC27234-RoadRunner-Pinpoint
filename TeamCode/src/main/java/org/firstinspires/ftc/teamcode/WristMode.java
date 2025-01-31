package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.AutoVariables.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WristMode {
    //************************************************************
    //**** Create Class and Methods for Wrist control
        //Define the Servo
        private Servo wrist;

        public WristMode(HardwareMap hardwareMap) {
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
                return false;
            }
        }

        public Action wristCollect() {
            return new WristCollect();
        }

        //Class to Put Wrist in Specimen Score Position
        public class WristScoreSpecimen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(WRIST_SCORE_SPECIMEN);
                return false;
            }
        }

        public Action wristScoreSpecimen() {
            return new WristCollect();
        }

        //Class to Put Wrist in Specimen Hang Position
        public class WristHangSpecimen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(WRIST_HANG_SPECIMEN);
                return false;
            }
        }

        public Action wristHangSpecimen() {
            return new WristHangSpecimen();
        }

}
