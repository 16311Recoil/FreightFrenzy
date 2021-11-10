package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

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
    private final double MAG_ON = 0.55;
    private final double MAG_OFF = 0;
    // private final double DUCK_POWER = 0;
    // private final int DUCK_TIME = 0;
    private final double ARM_POWER = 0.1;
    private final double MOTOR_ARM_GEAR_RATIO = 14 / 32.0; // 0.25 = The motor rotation is 1/4 of the resulting arm rotation
    private final double ARM_LENGTH = 15;
    private final double ROBOT_HEIGHT = 16;
    // private final double ARM_FLOOR_ANGLE = Math.toDegrees(Math.acos(ARM_LENGTH / ROBOT_HEIGHT)); // Angle at which the arm would be touching the floor
    private final double HEIGHT_LOWER_BOUND = 6;
    private final double DEGREE_TO_ENCODER_TICK = 537.7 / 360;
    // Robot height: 16

    // Tele-Op Utility
    private ElapsedTime timer = new ElapsedTime();
    private final int TOP_BOUND = 290;
    private final int LOW_BOUND = 0;
    private double goalEncoder = 0;
    private boolean stickPressed = false;
    private double prevTime = 0;

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
        armRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        armRotator.setPower(0.7);
        armRotator.setTargetPosition(0);
        armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        linear_OpMode.telemetry.addLine("Manipulator Init Completed - Linear");
        linear_OpMode.telemetry.update();
    }

    public Manipulator(OpMode opMode) {
        iterative_OpMode = opMode;

        // flyWheel = opMode.hardwareMap.get(DcMotorEx.class,"duck_wheel");
        armRotator = opMode.hardwareMap.get(DcMotorEx.class,  "armRotator");
        grabber = opMode.hardwareMap.get(Servo.class,   "grabber");
        magSwitch = opMode.hardwareMap.get(Servo.class, "magSwitch");

        // flyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        armRotator.setDirection(DcMotorSimple.Direction.FORWARD);

        // flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armRotator.setPower(0.7);
        armRotator.setTargetPosition(0);
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

    public void goToPosition (int position){
        armRotator.setTargetPosition(position);
    }

    public void goToLevel(double level) {
        // First, make sure we've received a level that can actually be reached
        if (level < HEIGHT_LOWER_BOUND || level > ROBOT_HEIGHT + ARM_LENGTH)
            return;

        armRotator.setTargetPosition(calcPosForLevelFrom0(level));
        armRotator.setPower(ARM_POWER);
    }

    public int calcPosForLevelFrom0(double level){
        return calcPosForLevel(HEIGHT_LOWER_BOUND) - calcPosForLevel(level);
    }

    public int calcPosForLevel(double level) {
        double c = ROBOT_HEIGHT;
        double new_target = toDeg(Math.asin(Math.abs(level - c) / ARM_LENGTH) * (level < c ? -1 : 1));
        new_target = 180 - new_target;
        // Adjust for gear ratios and encoder bullshit
        return (int) (new_target / MOTOR_ARM_GEAR_RATIO * DEGREE_TO_ENCODER_TICK);
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

    public double getEncoder(){
        return armRotator.getCurrentPosition();
    }

    public void teleOpAltContros(double gamepad){
        if (gamepad < 0){
            armRotator.setTargetPosition(LOW_BOUND);
        }
        armRotator.setTargetPosition((int)Math.round(gamepad * (TOP_BOUND - LOW_BOUND)));
    }

    public void teleOpControls(double gamepad, boolean a, boolean b){
        if (gamepad != 0){
            if (!stickPressed){
                timer.reset();
                prevTime = 0;
            }
            stickPressed = true;

            if ((timer.seconds() - prevTime) > 0.01){

                if (gamepad > 0){
                    if (goalEncoder < TOP_BOUND){
                        goalEncoder += 5 * gamepad;
                    }
                }

                if (gamepad < 0){
                    if (goalEncoder > LOW_BOUND){
                        goalEncoder += 5 * gamepad;
                    }
                }

                prevTime = timer.seconds();
            }
        }
        else {
            timer.reset();
            stickPressed = false;
        }

        magControl(a, b);

        armRotator.setTargetPosition((int)goalEncoder);
        iterative_OpMode.telemetry.addData("timer", timer.seconds());
        iterative_OpMode.telemetry.addData("goal", goalEncoder);
        iterative_OpMode.telemetry.addData("prev Timer", prevTime);
        iterative_OpMode.telemetry.addData("stickPressed", stickPressed);
    }

    public void magControl(boolean a, boolean b){
        if (a){
            magGrab();
        }
        if (b){
            magRelease();
        }
    }
}
