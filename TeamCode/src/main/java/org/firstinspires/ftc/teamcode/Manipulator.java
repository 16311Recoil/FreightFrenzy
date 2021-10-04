package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Manipulator {
    DcMotor flyWheel;

    Servo shoulder, turret;
    Servo grabber;
    Servo magSwitch;
    // TODO: Get actual arm measurements
    double height = 10;

    public Manipulator(OpMode opMode) {
        flyWheel = opMode.hardwareMap.get(DcMotor.class,"duck_wheel");
        shoulder = opMode.hardwareMap.get(Servo.class,  "servo180");
        grabber = opMode.hardwareMap.get(Servo.class,   "grabbyGrabGrab");
        magSwitch = opMode.hardwareMap.get(Servo.class, "magSwitch");
        turret = opMode.hardwareMap.get(Servo.class,    "turret");

        opMode.telemetry.addLine("Manipulator Init Completed");
        opMode.telemetry.update();
    }

    // --- Low-level Arm Control Functions -- //

    public void SetArmAngle(double raise_angle, double turret_angle){
        shoulder.setPosition(raise_angle);
        turret.setPosition(turret_angle);
    }

    public void MechGrab(){
        grabber.setPosition(180);
    }

    public void MechRelease(){
        grabber.setPosition(0);
    }

    public void MagGrab() {
        magSwitch.setPosition(180);
    }

    public void MagRelease(){
        magSwitch.setPosition(0);
    }

    // --- Abstracted Arm Control Functions --- //

    /**
     * Enable all holds
     */
    public void Grab(){
        MechGrab();
        MagGrab();
    }

    /**
     * Release all holds
     */
    public void Release(){
        MechRelease();
        MagRelease();
    }

    /**
     * Moves arm to specific position in XYZ coordinates relative to the robot
     * @param x distance in front of
     * @param y distance to the side of
     * @param z height of
     */
    public void MoveArmTo(double x, double y, double z){

        double azimuth = Math.atan2(y, x);
        double dist_to = Math.sqrt(x * x + y * y);
        double elevation = Math.atan2(z - height, dist_to);
        SetArmAngle(elevation, azimuth);
    }

    // --- Duck Functions --- //

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
