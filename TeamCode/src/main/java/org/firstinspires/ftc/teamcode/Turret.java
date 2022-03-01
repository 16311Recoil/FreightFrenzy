package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.euclidean.twod.Line;

import java.util.concurrent.BrokenBarrierException;

public class Turret {
    private LinearOpMode linear_OpMode;
    private OpMode iterative_OpMode;
    ColorSensor colorSensor;
    DcMotor turretMotor;

    private int targetAngle = 0;

    private final double ROTATION_SPEED = 0;
    double ENCODER_TO_ANGLE_RATIO = 1.24444;  //0.4643

    //teleo-op utility
    private PID pid = new PID();

    private ElapsedTime timer = new ElapsedTime();
    private int TOP_BOUND = 480;
    private int LOW_BOUND = -100;
    private int RED_LEFT_TOP = 105;
    private int RED_LEFT_BOTTOM = 105;
    private int RED_MIDDLE_BOTTOM = 120;
    private int RED_RIGHT_BOTTOM = 80;
    private int BLUE_LEFT_TOP = 480;
    private int BLUE_LEFT_BOTTOM = 480;
    private int BLUE_MIDDLE_BOTTOM = 440;
    private int BLUE_RIGHT_BOTTOM = 420;

    private boolean blueSide;
    private double goalEncoder = 0;
    private double goalAngle = 0;
    private boolean stickPressed = false;
    private double prevTime = 0;

    private double K_P = 0.025;
    private double K_I = 0;
    private double K_D = 0.0;

    private int ENCODER_TOLERANCE = 3;

    private boolean turretLock = true;

    public Turret(LinearOpMode opMode){
        linear_OpMode = opMode;

        turretMotor = opMode.hardwareMap.get(DcMotor.class, "turret_motor");
        // get a reference to our ColorSensor object.
        colorSensor = opMode.hardwareMap.get(ColorSensor.class, "sensor_color");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.5);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        opMode.telemetry.addLine("Turret Init Completed - Linear");
        opMode.telemetry.update();
    }

    public Turret(OpMode opMode){
        iterative_OpMode = opMode;

        turretMotor = opMode.hardwareMap.get(DcMotor.class, "turret_motor");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.5);
        // get a reference to our ColorSensor object.
        colorSensor = opMode.hardwareMap.get(ColorSensor.class, "sensor_color");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        opMode.telemetry.addLine("Turret Init Completed - Iterative");
        opMode.telemetry.update();
    }

    public void setDirection(int angle){
        targetAngle = angle;
    }

    public void setTurretMode(boolean powerMode){
        if (powerMode){
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        else {
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void setTurretPower (double power){
        turretMotor.setPower(power);
    }

    public void update(double robotDirection){
        if (turretLock)
            turretMotor.setTargetPosition((int)(nearestAngleToTurret((int)(targetAngle - robotDirection)) / ENCODER_TO_ANGLE_RATIO));
    }

    public boolean toggleTurretLock(){
        turretLock = !turretLock;
        return turretLock;
    }

    // TODO: Rewrite this function
    // Take the angle we want and add 360 until its the closest possible to the current turret angle
    // to have continuous fluid movement
    public int nearestAngleToTurret(int angle){
        int turretAngle = (int)(turretMotor.getCurrentPosition() * ENCODER_TO_ANGLE_RATIO);

        while (Math.abs(turretAngle - angle) >= 360){
            angle += 360 * Math.signum(turretAngle - angle);
        }

        return angle;
    }

    public int normalizeAngle(int angle){
        return ((angle % 360) + 360) % 360;
    }

    public void setPosition(int pos) {turretMotor.setTargetPosition(pos);}

    public int getPosition(){
        return turretMotor.getCurrentPosition();
    }

    public void teleOpControls(double gamepad, boolean x, boolean a, boolean dpadRight, boolean dpadDown, boolean dpadLeft, boolean dpadUp){

        if (x){
            TOP_BOUND = calibrationMode(gamepad);
            setLOW_BOUND(TOP_BOUND);
            goalEncoder = turretMotor.getCurrentPosition();
        }

        else if (gamepad != 0){
            if (!stickPressed){
                timer.reset();
                prevTime = 0;
            }
            stickPressed = true;

            if ((timer.seconds() - prevTime) > 0.01){

                if (gamepad > 0){
                    if (goalEncoder < TOP_BOUND){
                        goalEncoder += 8 * gamepad;
                    }
                }

                if (gamepad < 0){
                    if (goalEncoder > LOW_BOUND){
                        goalEncoder += 8 * gamepad;
                    }
                }

                prevTime = timer.seconds();
            }
        }
        else {
            timer.reset();
            stickPressed = false;
        }
        //turretMotor.setTargetPosition((int)goalEncoder);
        if (!x){
            if(a){
                if(blueSide){
                    if(dpadDown){
                        goalEncoder = LOW_BOUND + BLUE_MIDDLE_BOTTOM;
                    }
                    else if(dpadRight){
                        goalEncoder = LOW_BOUND + BLUE_RIGHT_BOTTOM;

                    }
                    else if(dpadLeft){
                        goalEncoder = LOW_BOUND + BLUE_LEFT_BOTTOM;
                    }
                    else if(dpadUp){
                        goalEncoder = LOW_BOUND + BLUE_LEFT_TOP;
                    }
                }
                else{
                    if(dpadDown){
                        goalEncoder = LOW_BOUND + RED_MIDDLE_BOTTOM;
                    }
                    else if(dpadRight){
                        goalEncoder = LOW_BOUND + RED_RIGHT_BOTTOM;

                    }
                    else if(dpadLeft){
                        goalEncoder = LOW_BOUND + RED_LEFT_BOTTOM;
                    }
                    else if(dpadUp){
                        goalEncoder = LOW_BOUND + RED_LEFT_TOP;
                    }
                }


            }

            moveTurretPID((int) goalEncoder);

        }
        iterative_OpMode.telemetry.addData("timer", timer.seconds());
        iterative_OpMode.telemetry.addData("Turret goal", goalEncoder);
        iterative_OpMode.telemetry.addData("prev Timer", prevTime);
        iterative_OpMode.telemetry.addData("stickPressed", stickPressed);
        iterative_OpMode.telemetry.update();
    }

    public void teleOpControlsTest(double gamepad, double rightTrigger, double leftTrigger, boolean x, boolean a, boolean dpadRight, boolean dpadDown, boolean dpadLeft, boolean dpadUp){
        double moveTurret = (gamepad - leftTrigger + rightTrigger);
        if (x){
            TOP_BOUND = calibrationMode(moveTurret);
            setLOW_BOUND(TOP_BOUND);
            goalEncoder = turretMotor.getCurrentPosition();
        }

        else if (moveTurret != 0){
            if (!stickPressed){
                timer.reset();
                prevTime = 0;
            }
            stickPressed = true;

            if ((timer.seconds() - prevTime) > 0.01){

                if (moveTurret > 0){
                    if (goalEncoder < TOP_BOUND){
                        goalEncoder += 8 * moveTurret;
                    }
                }

                if (moveTurret < 0){
                    if (goalEncoder > LOW_BOUND){
                        goalEncoder += 8 * moveTurret;
                    }
                }

                prevTime = timer.seconds();
            }
        }
        else {
            timer.reset();
            stickPressed = false;
        }
        //turretMotor.setTargetPosition((int)goalEncoder);
        if (!x){
            if(a){
                if(blueSide){
                    if(dpadDown){
                        goalEncoder = LOW_BOUND + BLUE_MIDDLE_BOTTOM;
                    }
                    else if(dpadRight){
                        goalEncoder = LOW_BOUND + BLUE_RIGHT_BOTTOM;

                    }
                    else if(dpadLeft){
                        goalEncoder = LOW_BOUND + BLUE_LEFT_BOTTOM;
                    }
                    else if(dpadUp){
                        goalEncoder = LOW_BOUND + BLUE_LEFT_TOP;
                    }
                }
                else{
                    if(dpadDown){
                        goalEncoder = LOW_BOUND + RED_MIDDLE_BOTTOM;
                    }
                    else if(dpadRight){
                        goalEncoder = LOW_BOUND + RED_RIGHT_BOTTOM;

                    }
                    else if(dpadLeft){
                        goalEncoder = LOW_BOUND + RED_LEFT_BOTTOM;
                    }
                    else if(dpadUp){
                        goalEncoder = LOW_BOUND + RED_LEFT_TOP;
                    }
                }


            }

            moveTurretPID((int) goalEncoder);

        }
        iterative_OpMode.telemetry.addData("timer", timer.seconds());
        iterative_OpMode.telemetry.addData("Turret goal", goalEncoder);
        iterative_OpMode.telemetry.addData("prev Timer", prevTime);
        iterative_OpMode.telemetry.addData("stickPressed", stickPressed);
        iterative_OpMode.telemetry.update();
    }
    public void teleOpColorControls(double gamepad, boolean x, boolean a, boolean dpadRight, boolean dpadDown, boolean dpadLeft, boolean dpadUp){

        if (x){
            TOP_BOUND = calibrationMode(gamepad);
            setLOW_BOUND(TOP_BOUND);
            goalAngle = getHue();
        }

        else if (gamepad != 0){
            if (!stickPressed){
                timer.reset();
                prevTime = 0;
            }
            stickPressed = true;

            if ((timer.seconds() - prevTime) > 0.01){

                if (gamepad > 0){
                    if (goalAngle < TOP_BOUND){
                        goalAngle += 8 * gamepad;
                    }
                }

                if (gamepad < 0){
                    if (goalAngle > LOW_BOUND){
                        goalAngle += 8 * gamepad;
                    }
                }

                prevTime = timer.seconds();
            }
        }
        else {
            timer.reset();
            stickPressed = false;
        }
        //turretMotor.setTargetPosition((int)goalEncoder);
        if (!x){
            if(a){
                if(blueSide){
                    if(dpadDown){
                        goalAngle = LOW_BOUND + BLUE_MIDDLE_BOTTOM;
                    }
                    else if(dpadRight){
                        goalAngle = LOW_BOUND + BLUE_RIGHT_BOTTOM;

                    }
                    else if(dpadLeft){
                        goalAngle = LOW_BOUND + BLUE_LEFT_BOTTOM;
                    }
                    else if(dpadUp){
                        goalAngle = LOW_BOUND + BLUE_LEFT_TOP;
                    }
                }
                else{
                    if(dpadDown){
                        goalAngle = LOW_BOUND + RED_MIDDLE_BOTTOM;
                    }
                    else if(dpadRight){
                        goalAngle = LOW_BOUND + RED_RIGHT_BOTTOM;

                    }
                    else if(dpadLeft){
                        goalAngle = LOW_BOUND + RED_LEFT_BOTTOM;
                    }
                    else if(dpadUp){
                        goalAngle = LOW_BOUND + RED_LEFT_TOP;
                    }
                }


            }

            moveTurretPID((int) goalAngle);

        }
        iterative_OpMode.telemetry.addData("timer", timer.seconds());
        iterative_OpMode.telemetry.addData("Turret goal", goalAngle);
        iterative_OpMode.telemetry.addData("prev Timer", prevTime);
        iterative_OpMode.telemetry.addData("stickPressed", stickPressed);
        iterative_OpMode.telemetry.update();
    }

    public void teleOpNewTurretControls(double gamepad, double rightTrigger, double leftTrigger, boolean x, boolean a, boolean dpadRight, boolean dpadDown, boolean dpadLeft, boolean dpadUp){
        double moveTurret = (gamepad + leftTrigger - rightTrigger);

        if (moveTurret != 0){
            if (!stickPressed){
                timer.reset();
                prevTime = 0;
            }
            stickPressed = true;

            if ((timer.milliseconds() - prevTime) > 1){

                if (x){
                    K_P = 0.02;
                    goalAngle += 14 * moveTurret;
                }
                else{
                    K_P = 0.01;
                    goalAngle += 7 * moveTurret;
                }


                prevTime = timer.milliseconds();
            }
        }
        else {
            timer.reset();
            stickPressed = false;
        }


        iterative_OpMode.telemetry.addData("goalAngle", goalAngle);
        iterative_OpMode.telemetry.addData("turPos", turretMotor.getCurrentPosition());
        iterative_OpMode.telemetry.addData("leftTrigger", leftTrigger);
        iterative_OpMode.telemetry.addData("rightTrigger", rightTrigger);

        moveTurretPID( (int) goalAngle);
    }


    public int calibrationMode(double gamepad){
        turretMotor.setPower(gamepad * 0.5);
        return turretMotor.getCurrentPosition();
    }

    public void moveTurretPID(int encoderTarget){
        int currentPos = getPosition();
        iterative_OpMode.telemetry.addData("curPos", currentPos);
        pid.setConstants(K_P, K_I,K_D, encoderTarget);
        if (Math.abs(encoderTarget - currentPos) > ENCODER_TOLERANCE){
            turretMotor.setPower(pid.loop(currentPos, timer.milliseconds()));
        }
        else{
            turretMotor.setPower(0);
        }

    }


    public void resetEncoder(){
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public float getHue(){
        float hsvValues[] = {0F,0F,0F};
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        return hsvValues[0];
    }
    public void setTOP_BOUND(int TOP){
        TOP_BOUND = TOP;
    }

    public void setLOW_BOUND(int HIGH_BOUND){
        LOW_BOUND = HIGH_BOUND - 580;
    }
    public void setAlliance(boolean alliance){
        blueSide = alliance;
    }
}
