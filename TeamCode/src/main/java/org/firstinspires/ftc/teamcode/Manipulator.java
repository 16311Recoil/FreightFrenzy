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

    private boolean grabEnabled = false;
    private boolean magEnabled = false;

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
    private final double ARM_POWER = 0.5;
    private final int ARM_FLOOR_ANGLE = 20;
    private final double MOTOR_ARM_GEAR_RATIO = 0.25; // 0.25 = The motor gear is 1/4 the size of the resulting arm movement
    private final double ARM_LENGTH = 10;

    private final double[] LEVELS = {0, 0, 0, 0};

    public Manipulator(LinearOpMode linear_OpMode) {
        this.linear_OpMode = linear_OpMode;

        flyWheel = linear_OpMode.hardwareMap.get(DcMotorEx.class,"duck_wheel");
        armRotator = linear_OpMode.hardwareMap.get(DcMotorEx.class,  "armRotator");
        grabber = linear_OpMode.hardwareMap.get(Servo.class,   "grabbyGrabGrab");
        magSwitch = linear_OpMode.hardwareMap.get(Servo.class, "magSwitch");

        flyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        armRotator.setDirection(DcMotorSimple.Direction.FORWARD);

        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armRotator.setPower(0);
        armRotator.setTargetPosition((int)(ARM_FLOOR_ANGLE / MOTOR_ARM_GEAR_RATIO));
        armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        linear_OpMode.telemetry.addLine("Manipulator Init Completed");
        linear_OpMode.telemetry.update();
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

        armRotator.setPower(0);
        armRotator.setTargetPosition((int)(ARM_FLOOR_ANGLE / MOTOR_ARM_GEAR_RATIO));
        armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        opMode.telemetry.addLine("Manipulator Init Completed");
        opMode.telemetry.update();
    }

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
        magSwitch.setPosition(MAG_OFF);
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


    // for STAS + ANYA
    // Complete!

    public void toggleGrabber(){
        if (grabEnabled) {
            mechRelease();
        } else {
            mechGrab();
        }
        grabEnabled = !grabEnabled;
    }

    public void toggleMagnet(){
        if (magEnabled) {
            magRelease();
        } else {
            magGrab();
        }
        magEnabled = !magEnabled;
    }

    public void setArmRotatorPower(double power){
        armRotator.setPower(power);
    }

    public void goToLevel(double level){
        // Use angles in rads for math funcs
        // Calculate arm circle lower and upper bounds
        double lower = -Math.sin(toRad(ARM_FLOOR_ANGLE)) * ARM_LENGTH - ARM_LENGTH,
                upper = lower + ARM_LENGTH * 2;

        double c = (lower + upper) / 2.0;

        // find the angle on the circle
        // FIXME: There's probably a better way to do this calculation
        double new_target = toDeg(Math.asin(Math.abs(level - c) / ARM_LENGTH)) * (level < c ? -1 : 1);
        if (ARM_FLOOR_ANGLE < 270){
            new_target = 180 - new_target;
        }
        // Adjust for gear ratios
        armRotator.setTargetPosition((int)(new_target / MOTOR_ARM_GEAR_RATIO));
        armRotator.setPower(ARM_POWER);
    }

    double toRad(double angle){
        return angle / 180.0 * Math.PI;
    }

    double toDeg(double rad){
        return rad * 180.0 / Math.PI;
    }

    double linearInterpolate(double a, double b, double t){
        return a + (b - a) * t;
    }

    double inverseLinearInterpolate(double min_val, double max_val, double res){
        // a + (b - a) * t = r
        // t = (r - a) / (b - a)
        return (res - min_val) / (max_val - min_val);
    }

    /**
     * Release grabber at an *exact height* (in inches)
     * @param level Height (in inches) to set
     */
    public void placeLevel(double level){
        goToLevel(level);
        release();
    }

    /**
     * Release grabber at a preset position
     * @param level_index The number of the preset level;
     *                    0: Floor;
     *                    1-3: Hub levels
     */
    public void placePresetLevel(int level_index){
        goToLevel(LEVELS[level_index]);
        release();
    }

    public void grabFloor(){
        release();
        goToLevel(0);
        grab();
    }
}
