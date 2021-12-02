package org.firstinspires.ftc.teamcode;

// robot class

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.teamcode.Drivetrain;

import java.util.Timer;

public class Crab {

    Drivetrain drivetrain;
    Manipulator manip;
    Turret turret;
    Sensors sensors;

    private OpMode teleOp;
    private LinearOpMode auto;
    private double relativeHeading;
    private boolean triggerPressRight;
    private double[] multipliers = {0.2, 0.55, 1};
    private int multiCounter = 1;
    private double multiplier = 1;
    private boolean changeDpadDown, changeDpadUp, changeY = false;
    private State teleOpState;

    private ElapsedTime duckTimer = new ElapsedTime();

    private enum State {
        NOT_DUCK,
        DUCK;
    }

    /*
      --------------------
      |                  |
      | BLEG |SSH | RLEG |
      |    > |    | <    |
      |   ===========    |
      |        ^         |
      |       Hat        |
    | |🦀              🦀|
    | |   BSH      RSH   |
    | |🦀              🦀|
    | O------------------O
    (0, 0)-----
     */

    final double HAT_X_BLUE = 13.7, HAT_X_RED = 0;
    final double HAT_Y = 0;
    final double BLUE_LEG_X = 0, RIGHT_LEG_X = 0;
    final double LEG_Y = 0;

    static class Hub{
        double x;
        double y;
        Hub(double x, double y){
            this.x = x;
            this.y = y;
        }
    }
    final Hub
            BLUE_HUB = new Hub(0, 0),
            RED_HUB = new Hub(0, 0),
            SHARED_HUB = new Hub(0, 0);

    Hub team_hub;
    double x = 0, y = 0;

    boolean alliance; // true = blue

    public Crab(LinearOpMode opMode){

        auto = opMode;

        drivetrain = new Drivetrain(opMode);
        manip = new Manipulator(opMode);
        turret = new Turret(opMode);
        sensors = new Sensors(opMode);

        opMode.telemetry.addLine("Crabtrain Init Completed - Linear");
        opMode.telemetry.update();
    }

    public Crab(OpMode opMode) {
        teleOp = opMode;

        drivetrain = new Drivetrain(opMode);
        manip = new Manipulator(opMode);
        turret = new Turret(opMode);
        sensors = new Sensors(opMode);

        teleOpState = State.NOT_DUCK;

        opMode.telemetry.addLine("Crabtrain Init Completed - Iterative");
        opMode.telemetry.update();
    }


    // --- Auto Functions --- //

    /**
     * Send off robot to reach a target state
     * @param targetX Resulting x position of intake
     * @param targetY Resulting y position of intake
     * @param targetOrientation Resulting robot orientation in degrees
     * @param grab Grab or release object?
     * @param grabLevel Level to release at; 0: Floor; 1-3: Hub levels
     */
    public void executeAction(double targetX, double targetY, double targetOrientation, boolean grab, int grabLevel){
        driveTo(targetX, targetY, targetOrientation);
        if (grab){
            manip.grabFloor();
        }
        else {
            manip.placePresetLevel(grabLevel);
        }
    }

    public void driveTo(double x, double y, double orientation){

    }

    public void grabTeamFreight(){

    }

    // --- Utility Functions --- //
    public double angleTowards(double x, double y){
        return Math.atan2(y - this.y, x - this.x);
    }

    public void toggleSpeed() {
        if ((teleOp.gamepad1.dpad_down && !changeDpadDown) && multiCounter > 0) {
            multiCounter--;
        }
        else if ((teleOp.gamepad1.dpad_up && !changeDpadUp) && multiCounter < 2) {
            multiCounter++;
        }
        multiplier = multipliers[multiCounter];



        changeDpadDown = teleOp.gamepad1.dpad_down;
        changeDpadUp = teleOp.gamepad1.dpad_up;

        teleOp.telemetry.addData("power", multiplier);
    }

    public void teleOpControlsBrenden(double init_Heading){
        double currentAngle = sensors.getFirstAngle();

        //drivetrain.moveTeleOp_Plus(gamepad1.left_stick_x, gamepad1.left_stick_y, drivetrain.lockHeadingAngle(relativeHeading - init_Heading, sensor.getFirstAngle()), 1, 1); //lock angle
        //drivetrain.moveGyroTeleOp_Plus(gamepad1.left_stick_x, gamepad1.left_stick_y, drivetrain.lockHeadingAngle(drivetrain.lockNearestX(currentAngle - init_Heading), currentAngle), 1, 1, currentAngle); //lock x-mode
        if (teleOp.gamepad1.y && !changeY){
            teleOpState = State.DUCK;
            duckTimer.reset();
        }

        if (teleOpState.equals(State.NOT_DUCK)){
            if (teleOp.gamepad1.right_trigger > 0){
                drivetrain.moveGyroTeleOp_Plus(teleOp.gamepad1.right_stick_x, teleOp.gamepad1.right_stick_y, drivetrain.lockHeadingAngle(drivetrain.lockNearestX(currentAngle - init_Heading), currentAngle - init_Heading), multiplier, multiplier, currentAngle - init_Heading); //lock x-mode
            }
            else if (teleOp.gamepad1.left_trigger > 0){
                if (!triggerPressRight){
                    relativeHeading = currentAngle;
                }
                triggerPressRight = true;
                drivetrain.moveGyroTeleOp_Plus(teleOp.gamepad1.right_stick_x, teleOp.gamepad1.right_stick_y, drivetrain.lockHeadingAngle(relativeHeading - init_Heading, currentAngle - init_Heading), multiplier, multiplier, currentAngle - init_Heading); //lock angle

            }
            else { //regular driving
                triggerPressRight = false;
                drivetrain.moveGyroTeleOp_Plus(teleOp.gamepad1.right_stick_x, teleOp.gamepad1.right_stick_y, teleOp.gamepad1.left_stick_x, multiplier, multiplier, currentAngle - init_Heading);
            }

            manip.teleOpControls(-teleOp.gamepad2.left_stick_y, teleOp.gamepad2.a, teleOp.gamepad2.b, teleOp.gamepad2.dpad_down, teleOp.gamepad2.dpad_left, teleOp.gamepad2.dpad_up, teleOp.gamepad2.dpad_right);
            turret.teleOpControls(-teleOp.gamepad2.right_stick_x, teleOp.gamepad2.right_bumper);
            toggleSpeed();
        }

        else{
            doDuck(init_Heading, duckTimer.seconds());
        }
    }

    public void doDuck(double init_Heading, double time){
        manip.goToPosition(240);
        while (time < 1.6){
            drivetrain.spinDuck(0.6, 0.2, 1.35 * Math.PI, sensors.getFirstAngle() - init_Heading, time, alliance);
        }
    }

    /**
     * Ram the robot into the wall
     * @param speed direction along the wall to move.
     *                  >0 = towards warehouse; <0 = towards team parking
     */
    public void violentlyRamWall(double speed, long time) throws InterruptedException{
        if (alliance){
            drivetrain.setMotorPowers(speed, 0.1, 0.1, speed);
        }
        else {
            drivetrain.setMotorPowers(-speed, -0.1, -0.1, -speed);
        }
        Thread.sleep(time);
        drivetrain.setMotorPowers(0, 0, 0, 0);
    }

    public void violentlyRamDucks() throws InterruptedException{
        violentlyRamWall(-1, 5000);
        double mult = alliance ? 1 : -1;
        drivetrain.setMotorPowers(0, 0.5 * mult, 0, 0.5 * mult);
        violentlyRamWall(-1, 1000);
        //drivetrain.spinDuck(1, 0.1, (1.5 + 0.25 * mult) * Math.PI, Math.PI * (0.5 - 0.5 * mult), true);
    }


    public void teleOpControlsAditya(double init_Heading){
    }

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }

    public Manipulator getManip() {
        return manip;
    }

    public Turret getTurret() {
        return turret;
    }

    public Sensors getSensors() {
        return sensors;
    }

    public void setAlliance(boolean blueSide){
        if (blueSide)
            team_hub = BLUE_HUB;
        else
            team_hub = RED_HUB;
        alliance = blueSide;
    }


}
