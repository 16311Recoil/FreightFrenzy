package org.firstinspires.ftc.teamcode;

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

    DcMotor armRotator;
    Servo grabber;
    Servo magSwitch;

    // All measurements in inches or degrees

    private final double GRAB = 180;
    private final double UNGRAB = 0;
    private final double MAG_ON = 180;
    private final double MAG_OFF = 0;
    // private final double DUCK_POWER = 0;
    // private final int DUCK_TIME = 0;
    private final double ARM_POWER = 0.5;
    // TODO: FIND ARM GEAR RATIO
    private final double MOTOR_ARM_GEAR_RATIO = 1; // 0.25 = The motor rotation is 1/4 of the resulting arm rotation
    private final double ARM_LENGTH = 15;
    private final double ROBOT_HEIGHT = 16;
    // private final double ARM_FLOOR_ANGLE = Math.toDegrees(Math.acos(ARM_LENGTH / ROBOT_HEIGHT)); // Angle at which the arm would be touching the floor
    private final double HEIGHT_LOWER_BOUND = 6;
    // Robot height: 16

    private final double[] LEVELS = {0, 3 + 2 + 0.5, 8.5 + 2 + 0.5, 14.75 + 2 + 0.5};

    public Manipulator(LinearOpMode opMode) {
        linear_OpMode = opMode;

        // flyWheel = linear_OpMode.hardwareMap.get(DcMotorEx.class,"duck_wheel");
        armRotator = opMode.hardwareMap.get(DcMotorEx.class,  "armRotator");
        grabber = opMode.hardwareMap.get(Servo.class,   "grabber");
        magSwitch = opMode.hardwareMap.get(Servo.class, "magSwitch");

        // flyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        armRotator.setDirection(DcMotorSimple.Direction.FORWARD);

        // flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armRotator.setPower(0);
        armRotator.setTargetPosition((int)(180 / MOTOR_ARM_GEAR_RATIO));
        armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        linear_OpMode.telemetry.addLine("Manipulator Init Completed - Linear");
        linear_OpMode.telemetry.update();
    }

    public Manipulator(OpMode opMode) {
        iterative_OpMode = opMode;

        // flyWheel = opMode.hardwareMap.get(DcMotorEx.class,"duck_wheel");
        armRotator = opMode.hardwareMap.get(DcMotorEx.class,  "armRotator");
        grabber = opMode.hardwareMap.get(Servo.class,   "grabbyGrabGrab");
        magSwitch = opMode.hardwareMap.get(Servo.class, "magSwitch");

        // flyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        armRotator.setDirection(DcMotorSimple.Direction.FORWARD);

        // flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armRotator.setPower(0);
        armRotator.setTargetPosition((int)(180 / MOTOR_ARM_GEAR_RATIO));
        armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        opMode.telemetry.addLine("Manipulator Init Completed - Iterative");
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
    /*
    public void grabDuck() throws InterruptedException{
        flyWheel.setPower(DUCK_POWER); // TODO: Figure out duck power
        Thread.sleep(DUCK_TIME);
        // TODO: find a way to wait- Its Thread.sleep() and you have to have the method throw InterupptedEception Error
        flyWheel.setPower(0);
    }
     */

    // --- Abstracted claw control functions --- //

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

    public void setArmMode(boolean powerMode){
        if (powerMode){
            armRotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        else {
            armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void setArmRotatorPower(double power){
        armRotator.setPower(power);
    }

    public void goToLevel(double level) {
        // First, make sure we've received a level that can actually be reached
        if (level < HEIGHT_LOWER_BOUND || level > ROBOT_HEIGHT + ARM_LENGTH)
            return ;

        // Calculate arm circle lower and upper bounds
        /* double lower = -Math.sin(toRad(ARM_FLOOR_ANGLE)) * ARM_LENGTH - ARM_LENGTH,
                upper = lower + ARM_LENGTH * 2; */

        double c = /* (lower + upper) / 2.0 */ ROBOT_HEIGHT;

        // find the angle on the circle
        double new_target = toDeg(Math.asin(Math.abs(level - c) / ARM_LENGTH)) * (level < c ? -1 : 1);
        new_target = 180 - new_target;

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
        goToLevel(HEIGHT_LOWER_BOUND);
        grab();
    }
}
