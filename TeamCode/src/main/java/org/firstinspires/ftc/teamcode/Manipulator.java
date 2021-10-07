package org.firstinspires.ftc.teamcode;

import androidx.annotation.BoolRes;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

public class Manipulator {
    private LinearOpMode linear_OpMode;
    private OpMode iterative_OpMode;


    //spoolWinder is a servo and shoulder is a motor, also gonna get rid of shoulder because your making a turret class
    //Need motors for lifting up down
    private DcMotor armRotator = null;
    private DcMotor flyWheel = null;
    private Servo spoolWinder, grabber, magSwitch;


    private double GRAB = 0;
    private double UNGRAB = 0;
    private double MAG_ON = 0;
    private double MAG_OFF = 0;
    private double DUCK_POWER = 0;


    //define all of your private variable at the top



    //if you want to make a class for the Clamp, just make a seperate class rather than make a
    // static class inside the manip class. I personally would just keep the methods in this class

    public Manipulator(LinearOpMode opMode){
        this.linear_OpMode = opMode;

        spoolWinder = linear_OpMode.hardwareMap.get(Servo.class,"lift_spool");
        grabber = linear_OpMode.hardwareMap.get(Servo.class,"grabbyGrabGrab");
        magSwitch = linear_OpMode.hardwareMap.get(Servo.class,"magSwitch");

        flyWheel = linear_OpMode.hardwareMap.get(DcMotor.class,"duck_wheel");
        armRotator = linear_OpMode.hardwareMap.get(DcMotor.class, "armRotator");

        flyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        armRotator.setDirection(DcMotorSimple.Direction.FORWARD);

        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Manipulator(OpMode opMode){
        this.iterative_OpMode = opMode;

        spoolWinder = iterative_OpMode.hardwareMap.get(Servo.class,  "lift_spool");
        grabber = iterative_OpMode.hardwareMap.get(Servo.class,   "grabbyGrabGrab");
        magSwitch = iterative_OpMode.hardwareMap.get(Servo.class, "magSwitch");

        flyWheel = iterative_OpMode.hardwareMap.get(DcMotor.class,    "duck_wheel");
        armRotator = iterative_OpMode.hardwareMap.get(DcMotor.class, "armRotator");

        flyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        armRotator.setDirection(DcMotorSimple.Direction.FORWARD);

        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    //use camel case so basically first word uncapital every other word capital ex: moveTo instead of MoveTo

    public void grab(){
        grabber.setPosition(GRAB); // just use variables at the top to make it easy to change
    }

    public void unGrab(){
        grabber.setPosition(UNGRAB); // just use variables at the top to make it easy to change; don't have to look for the method
    }

    public void magnetOn(){
        magSwitch.setPosition(MAG_ON);
    }

    public void magnetOff(){
        magSwitch.setPosition(MAG_OFF);
    }

    //just put it in the object contructor rather than a init method, never would need to seperately call init


    //method wouldn't exist in this class

    //detection isn't part of this class so no point in this method

    public void setFlyWheelPower(double power){
        flyWheel.setPower(power);
    }

    public void spinDuck() throws InterruptedException{ //not grabbing so changed name
        flyWheel.setPower(DUCK_POWER); // Power goes from 0 - 1. TODO: Figure out duck power
        Thread.sleep(100);// TOD: find a way to wait //here's the way to wait
        flyWheel.setPower(0);
    }


    //TODO for STAS + ANYA

    public void toggleGrabber(){
    }

    public void toggleMagnet(){
    }

    public void setArmRotatorPower(){

    }

    public void goToLevel(double level){
        //knowing that you are dealing with a motor for the armRotator,
        // figure out how you can stop the motor at the correct height for the floor, bottom, middle and top levels (0, 1, 2, 3)
        // (TIP: You need to stall out the motor once you get to the destired height)
        // (TIP: Use motorName.getCurrentPosition() to get the encoder value, but don't worry about converting to inches yet)
    }

    public void placeLevel(double level){
    }

    public void grabFloor(){
        unGrab();
        goToLevel(0);
        grab();
    }





}
