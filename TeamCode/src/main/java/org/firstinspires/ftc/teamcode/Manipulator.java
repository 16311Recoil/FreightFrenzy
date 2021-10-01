package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Manipulator {
    DcMotor flyWheel;

    static class Clamp{
        Servo shoulder;
        Servo grabber;
        Servo magSwitch;
        Servo extender;
        double clampBaseLength = 10, clampExtendedLength = 15;
        double height = 10;

        public Clamp(@NonNull OpMode opMode){
            shoulder = opMode.hardwareMap.get(Servo.class,  "servo180");
            grabber = opMode.hardwareMap.get(Servo.class,   "grabbyGrabGrab");
            magSwitch = opMode.hardwareMap.get(Servo.class, "magSwitch");
            extender = opMode.hardwareMap.get(Servo.class,  "extender");
        }
        public void MoveShoulderTo(double angle){
            shoulder.setPosition(angle);
        }

        public void SetExtenderLength(double length){
            extender.setPosition(180 * (length - clampBaseLength) / (clampExtendedLength - clampBaseLength));
        }

        public void Grab(){
            grabber.setPosition(180);
        }

        public void UnGrab(){
            grabber.setPosition(0);
        }

        public void MoveArmTo(double distance_to, double height_at){
            double height_dif = height_at - height;
            double angle = Math.atan2(height_at, distance_to);
            MoveShoulderTo(angle);

            double extensionDst = Math.sqrt(distance_to * distance_to + height_dif * height_dif) - clampBaseLength;
            SetExtenderLength(extensionDst);
        }

        public void GrabObjectAt(double distance_to){
            UnGrab();
            MoveArmTo(distance_to, 0);
            Grab();
        }

        public void ReleaseObjectAt(double distance_to, double height_at){
            MoveArmTo(distance_to, height_at);
            UnGrab();
        }

        public void ReleaseObjectAt(double distance_to){
            ReleaseObjectAt(distance_to, 0);
        }

    }

    Clamp clamp;

    public Manipulator(@NonNull OpMode opMode) {
        flyWheel = opMode.hardwareMap.get(DcMotor.class,"duck_wheel");
        clamp = new Clamp(opMode);
        opMode.telemetry.addLine("Manipulator Init Completed");
        opMode.telemetry.update();
    }

    public boolean DetectDuck(){
        return true; // TODO: Detect duck. Probably done in Austin's sensor class
    }

    public void GrabDucks(){
        // Made non-blocking with threading
        for (int i = 0; i < 8; i ++){
            while (DetectDuck()) { // Just in case
                GrabDuck();
            }
        }
    }

    public void GrabDuck(){
        flyWheel.setPower(5); // TODO: Figure out duck power
        // TODO: find a way to wait
        flyWheel.setPower(0);
    }
}
