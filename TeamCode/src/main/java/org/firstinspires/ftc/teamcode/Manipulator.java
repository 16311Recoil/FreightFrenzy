package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Manipulator {
    private LinearOpMode linear_OpMode;
    private OpMode iterative_OpMode;

    DcMotor flyWheel;

    DcMotor armRotator; // (previous shoulder) this is a motor not a servo
    Servo grabber;
    Servo magSwitch;

    //define all of your private variables at the top


    private final double GRAB = 0;
    private final double UNGRAB = 0;
    private final double MAG_ON = 0;
    private final double MAG_OFF = 0;
    private final double DUCK_POWER = 0;
    private final int DUCK_TIME = 0;

    public Manipulator(LinearOpMode opMode) { //Need a constructor for both LinearOpmode and Opmode
        this.linear_OpMode = opMode;

        flyWheel = opMode.hardwareMap.get(DcMotorEx.class,"duck_wheel");
        armRotator = opMode.hardwareMap.get(DcMotorEx.class,  "armRotator");
        grabber = opMode.hardwareMap.get(Servo.class,   "grabbyGrabGrab");
        magSwitch = opMode.hardwareMap.get(Servo.class, "magSwitch");

        flyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        armRotator.setDirection(DcMotorSimple.Direction.FORWARD);

        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        opMode.telemetry.addLine("Manipulator Init Completed");
        opMode.telemetry.update();
    }

    public Manipulator(OpMode opMode) {
        this.iterative_OpMode = opMode;

        flyWheel = opMode.hardwareMap.get(DcMotorEx.class,"duck_wheel");
        armRotator = opMode.hardwareMap.get(DcMotorEx.class,  "armRotator");
        grabber = opMode.hardwareMap.get(Servo.class,   "grabbyGrabGrab");
        magSwitch = opMode.hardwareMap.get(Servo.class, "magSwitch");

        flyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        armRotator.setDirection(DcMotorSimple.Direction.FORWARD);

        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        opMode.telemetry.addLine("Manipulator Init Completed");
        opMode.telemetry.update();
    }

    //use camel case so basically first word uncapital every other word capital ex: moveTo instead of MoveTo


    //Turret methods wouldn't exist in this class

    //Detection isn't part of this class either, that fuctionality would be added in the robot class probably

    // --- Low-level Arm Control Functions -- //

    public void mechGrab(){
        grabber.setPosition(GRAB);
    }

    public void mechRelease(){
        grabber.setPosition(UNGRAB);
    }

    public void magGrab() {
        magSwitch.setPosition(MAG_ON);
    }

    public void magRelease(){
        magSwitch.setPosition(0);
    }

    // --- Abstracted Arm Control Functions --- //

    /**
     * Enable all holds
     */
    public void grab(){
        mechGrab();
        magGrab();
    }

    /**
     * Release all holds
     */
    public void release(){
        mechRelease();
        magRelease();
    }

    // --- Duck Functions --- //

    public void grabDuck() throws InterruptedException{
        flyWheel.setPower(DUCK_POWER); // TODO: Figure out duck power
        Thread.sleep(DUCK_TIME);
        // TODO: find a way to wait- Its Thread.sleep() and you have to have the method throw InterupptedEception Error
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
        // Might to make multiple variables or other methods to properly do
    }

    public void placeLevel(double level){
        goToLevel(level);
        release();
    }

    public void grabFloor(){
        release();
        goToLevel(0);
        grab();
    }




}
