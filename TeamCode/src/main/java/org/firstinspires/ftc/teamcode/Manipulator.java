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
    private boolean upEnabled = false;
    private boolean armDown = false;

    DcMotorEx armRotator;

    Servo grabber;
    Servo magSwitch;
    Servo clawRotator;

    private enum ArmState{
        PICK_UP,
        HUB_DROP,
        ALLIANCE_DROP,
        DUCK,
        MOVE_DOWN,
        MOVE_UP,
        CUSTOM;
    }

    ArmState goalArmState;
    ArmState armState;
    ArmState prevArmState;

    private int STATE_TOLERANCE = 3;

    // All measurements in inches or scalars 0-1
    private final double UP = 0.36;
    private final double HALF_UP = 0.25;
    private final double DOWN = 0;
    private final double GRAB = .69;
    private final double UNGRAB = 0.0;
    private final double MAG_ON = 0.55;
    private final double MAG_OFF = 0;
    // private final double DUCK_POWER = 0;
    // private final int DUCK_TIME = 0;
    private final double ARM_POWER = 0.85;
    private final double MOTOR_ARM_GEAR_RATIO = 14 / 32.0; // 0.25 = The motor rotation is 1/4 of the resulting arm rotation
    private final double ARM_LENGTH = 15;
    private final double ROBOT_HEIGHT = 16;
    // private final double ARM_FLOOR_ANGLE = Math.toDegrees(Math.acos(ARM_LENGTH / ROBOT_HEIGHT)); // Angle at which the arm would be touching the floor
    private final double HEIGHT_LOWER_BOUND = 6;
    private final double DEGREE_TO_ENCODER_TICK = 537.7 / 360;
    // Robot height: 16

    // Tele-Op Utility
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime magTimer = new ElapsedTime();
    private ElapsedTime clawTimer = new ElapsedTime();
    private final int TOP_BOUND = -4600;
    private final int LOW_BOUND = 0;
    private double PICK_UP = -400;
    private double goalEncoder = 0;
    private boolean stickPressed = false;
    private double prevTime = 0;

    private boolean changeX = false;
    private boolean changeY = false;
    private boolean changeA1 = false;

    private final double[] LEVELS = {6, 3 + 2 + 0.5, 8.5 + 2 + 0.5, 14.75 + 2 + 0.5};

    public Manipulator(LinearOpMode opMode) {
        linear_OpMode = opMode;

        // flyWheel = linear_OpMode.hardwareMap.get(DcMotorEx.class,"duck_wheel");
        armRotator = opMode.hardwareMap.get(DcMotorEx.class,  "armRotator");
        grabber = opMode.hardwareMap.get(Servo.class,   "grabber");
        magSwitch = opMode.hardwareMap.get(Servo.class, "magSwitch");
        clawRotator = opMode.hardwareMap.get(Servo.class, "clawRotator");

        // flyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        armRotator.setDirection(DcMotorSimple.Direction.FORWARD);

        // flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        armRotator.setPower(ARM_POWER);
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
        clawRotator = opMode.hardwareMap.get(Servo.class, "clawRotator");

        // flyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        armRotator.setDirection(DcMotorSimple.Direction.FORWARD);

        // flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armRotator.setPower(ARM_POWER);
        armRotator.setTargetPosition(0);
        armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        clawRotator.setPosition(0);

        armState = ArmState.CUSTOM;


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

    public void rotateClawDown(){clawRotator.setPosition(DOWN);}
    
    public void rotateClawUp(){clawRotator.setPosition(UP);}
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

    public void toggleArmDown(){
        if (armDown) {
            goalEncoder = -1200;
        }
        else {
            goalEncoder = PICK_UP;
        }
        armDown = !armDown;
    }

    public void toggleClawRotate(){
        if (upEnabled) {
            rotateClawDown();
        }
        else {
            rotateClawUp();
        }
        upEnabled = !upEnabled;
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

    /*
    public void goToPosition (int position){
        armRotator.setTargetPosition(position);
    }
     */

    public void goToPosition(int targetPos, int turretPos){
        double bias = getBias(turretPos);
        armRotator.setTargetPosition((int)(targetPos - bias));
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
        //release();
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

    public void teleOpControls(double gamepad, boolean a, boolean b, boolean x, boolean y, boolean down, boolean left, boolean up, boolean right){
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

        magControl(b);
        clawControl(y);
        clawRotateControl(x);
        if(!a) {
            if (down) {
                goalEncoder = 40;
            }
            if (up) {
                goalEncoder = 375;
            }
            if (right) {
                goalEncoder = 120;
            }
            if (left) {
                goalEncoder = 250;
            }
        }

        armRotator.setTargetPosition((int)goalEncoder);

        /*iterative_OpMode.telemetry.addData("timer", timer.seconds());
        iterative_OpMode.telemetry.addData("goal", goalEncoder);
        iterative_OpMode.telemetry.addData("prev Timer", prevTime);
        iterative_OpMode.telemetry.addData("stickPressed", stickPressed);
        iterative_OpMode.telemetry.addData("Encoder", armRotator.getCurrentPosition());
*/
    }

    public void teleOpNewControls(double gamepad, boolean a, boolean b, boolean x, boolean y, boolean down, boolean left, boolean up, boolean right, boolean stickButton, int turPos, boolean a1){
        double bias = getBias(turPos);

        if (stickButton){
            PICK_UP = goalEncoder;
        }

        if (gamepad != 0){
            if (!stickPressed){
                timer.reset();
                prevTime = 0;
            }
            stickPressed = true;

            if ((timer.seconds() - prevTime) > 0.01){

                if (gamepad > 0){
                    if (goalEncoder - bias > TOP_BOUND - bias){
                        goalEncoder -= 50 * gamepad;
                    }
                }

                if (gamepad < 0){
                    if (goalEncoder - bias < LOW_BOUND - bias){
                        goalEncoder -= 50 * gamepad;
                    }
                }

                prevTime = timer.seconds();
            }
        }
        else {
            timer.reset();
            stickPressed = false;
        }

        magControl(b);
        clawControl(y);
        clawRotateControl(x);
        armControl(a1);
        if(!a) {
            if (down) {
                goalEncoder = PICK_UP;
            }
            if (up) {
                goalEncoder = TOP_BOUND;
            }
            if (right) {
                goalEncoder = -1200;
            }
            if (left) {
                goalEncoder = -3100;
            }
        }
        armRotator.setTargetPosition((int)(goalEncoder - bias));


        iterative_OpMode.telemetry.addData("ArmGoal", goalEncoder);
        iterative_OpMode.telemetry.addData("ArmPos", armRotator.getCurrentPosition());
        iterative_OpMode.telemetry.addData("lowBound", LOW_BOUND - bias);
        iterative_OpMode.telemetry.addData("highBound", TOP_BOUND + bias);
        iterative_OpMode.telemetry.addData("bias", bias);

    }

    public double getBias(int turPos){
        return 0.7858 * turPos;
    }

    public int getBiasedPosition(int turPos){
        return (int)(goalEncoder - getBias(turPos));
    }

    public void teleOpControlsStateMachine (double gamepad, boolean a, boolean b, boolean x, boolean y, boolean down, boolean left, boolean up, boolean right){

        int curPos = armRotator.getCurrentPosition();

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
                goalArmState = ArmState.CUSTOM;
                prevTime = timer.seconds();
            }
        }
        else {
            timer.reset();
            stickPressed = false;
        }

        magControl(b);
        clawControl(y);
        clawRotateControl(x);

        if (goalArmState.equals(ArmState.CUSTOM)){
            setMoveDirectionConstants(goalEncoder, curPos);
        }

        if (down){
            goalArmState = ArmState.PICK_UP;
            goalEncoder = 15;

            switch (armState){
                case DUCK:
                    armRotator.setVelocityPIDFCoefficients(0,0,0,0);
                    armRotator.setPositionPIDFCoefficients(0);
                case ALLIANCE_DROP:
                    armRotator.setVelocityPIDFCoefficients(0,0,0,0);
                    armRotator.setPositionPIDFCoefficients(0);
                case HUB_DROP:
                    armRotator.setVelocityPIDFCoefficients(0,0,0,0);
                    armRotator.setPositionPIDFCoefficients(0);
                case CUSTOM:
                    setMoveDirectionConstants(goalEncoder, curPos);
                    break;
            }

        }
        if (up){
            goalArmState = ArmState.DUCK;
            goalEncoder = 375;

            switch (armState){
                case ALLIANCE_DROP:
                    armRotator.setVelocityPIDFCoefficients(0,0,0,0);
                    armRotator.setPositionPIDFCoefficients(0);
                case HUB_DROP:
                    armRotator.setVelocityPIDFCoefficients(0,0,0,0);
                    armRotator.setPositionPIDFCoefficients(0);
                case PICK_UP:
                    armRotator.setVelocityPIDFCoefficients(0,0,0,0);
                    armRotator.setPositionPIDFCoefficients(0);
                case CUSTOM:
                    setMoveDirectionConstants(goalEncoder, curPos);
                    break;
            }

        }
        if (right){
            goalArmState = ArmState.HUB_DROP;
            goalEncoder = 120;

            switch (armState){
                case DUCK:
                    armRotator.setVelocityPIDFCoefficients(0,0,0,0);
                    armRotator.setPositionPIDFCoefficients(0);
                case ALLIANCE_DROP:
                    armRotator.setVelocityPIDFCoefficients(0,0,0,0);
                    armRotator.setPositionPIDFCoefficients(0);
                case PICK_UP:
                    armRotator.setVelocityPIDFCoefficients(0,0,0,0);
                    armRotator.setPositionPIDFCoefficients(0);
                case MOVE_DOWN:
                    armRotator.setVelocityPIDFCoefficients(0,0,0,0);
                    armRotator.setPositionPIDFCoefficients(0);
                case CUSTOM:
                    setMoveDirectionConstants(goalEncoder, curPos);
                    break;
            }

        }
        if (left) {
            goalArmState = ArmState.ALLIANCE_DROP;
            goalEncoder = 240;

            switch (armState){
                case DUCK:
                    armRotator.setVelocityPIDFCoefficients(0,0,0,0);
                    armRotator.setPositionPIDFCoefficients(0);
                case HUB_DROP:
                    armRotator.setVelocityPIDFCoefficients(0,0,0,0);
                    armRotator.setPositionPIDFCoefficients(0);
                case PICK_UP:
                    armRotator.setVelocityPIDFCoefficients(0,0,0,0);
                    armRotator.setPositionPIDFCoefficients(0);
                case CUSTOM:
                    setMoveDirectionConstants(goalEncoder, curPos);
                    break;
            }
        }

        if (Math.abs(goalEncoder - curPos) < STATE_TOLERANCE){
            armState = goalArmState;
        }

        armRotator.setTargetPosition((int)goalEncoder);


        iterative_OpMode.telemetry.addData("timer", timer.seconds());
        iterative_OpMode.telemetry.addData("goal", goalEncoder);
        iterative_OpMode.telemetry.addData("prev Timer", prevTime);
        iterative_OpMode.telemetry.addData("stickPressed", stickPressed);
        iterative_OpMode.telemetry.addData("Encoder", curPos);
        iterative_OpMode.telemetry.addData("State", armState);
        iterative_OpMode.telemetry.addData("goalState", goalArmState);
        iterative_OpMode.telemetry.addData("prevState", prevArmState);


    }


    public void magControl( boolean b){

        if (magTimer.seconds() > 1.5){
            magGrab();
        }
        if (b){
            magRelease();
            magTimer.reset();
        }
    }

    public void armControl(boolean a1){
        if (a1 && !changeA1){
            toggleArmDown();
        }
        changeA1 = a1;
    }

    public void clawControl(boolean y){
        if(y && !changeY){
            toggleGrabber();
        }
        changeY = y;
    }

    public void clawRotateControl(boolean x){
        if (x && !changeX){
            toggleClawRotate();
        }
        if (iterative_OpMode.gamepad2.left_bumper){
            clawRotator.setPosition(HALF_UP);
        }
        changeX = x;
    }

    public void setMoveDirectionConstants(double goalEncoder, int curPos){
        if ((goalEncoder - curPos) > 0){
            armRotator.setVelocityPIDFCoefficients(0,0,0,0);
            armRotator.setPositionPIDFCoefficients(1);
        }
        else if ((goalEncoder - curPos) < 0){
            armRotator.setVelocityPIDFCoefficients(0,0,0,0);
            armRotator.setPositionPIDFCoefficients(0);
        }
    }

    public void setGoalEncoder(int val){
        goalEncoder = val;
    }

}
