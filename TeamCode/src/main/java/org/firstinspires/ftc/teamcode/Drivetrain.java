package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.util.ArrayUtils;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.Collections;

@Config
public class Drivetrain{

    private DcMotorEx f, r, l, b;
    private Servo ffOdom, sfOdom;
    private LinearOpMode linear_OpMode;
    private OpMode iterative_OpMode;
    private FtcDashboard dashboard;

    public static PIDCoefficients mPID = new PIDCoefficients(0.005, 0, 0.005);
    public static PIDCoefficients spinPID = new PIDCoefficients(0.08, 0.08, 0.08);
    public static double turnError = 0.02;

    private double FF_LOW = 0.64;
    private double SF_LOW = 0.45;
    private double ENCODER_IN_INCHES = 39.88422; // 2086.07567 for odom, 39.88422 for wheel encoders

    public Drivetrain(LinearOpMode opMode){
        this.linear_OpMode = opMode;

        f = this.linear_OpMode.hardwareMap.get(DcMotorEx.class, "f");
        r = this.linear_OpMode.hardwareMap.get(DcMotorEx.class, "r");
        l = this.linear_OpMode.hardwareMap.get(DcMotorEx.class, "l");
        b = this.linear_OpMode.hardwareMap.get(DcMotorEx.class, "b");

        ffOdom = this.linear_OpMode.hardwareMap.servo.get("ffOdom");
        sfOdom = this.linear_OpMode.hardwareMap.servo.get("sfOdom");

        l.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        f.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        l.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        f.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        b.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        l.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        f.setDirection(DcMotorSimple.Direction.FORWARD);
        r.setDirection(DcMotorSimple.Direction.REVERSE);
        l.setDirection(DcMotorSimple.Direction.REVERSE);
        b.setDirection(DcMotorSimple.Direction.FORWARD);

        ffOdom.setDirection(Servo.Direction.FORWARD);
        sfOdom.setDirection(Servo.Direction.FORWARD);

        for (LynxModule module : this.linear_OpMode.hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        ffOdom.setPosition(0);
        sfOdom.setPosition(0);

        // OH FUCKING HELL I WAS PUTTING IN THE FINIT/LINIT IN THE LINEAR ONE
        opMode.telemetry.addLine("Drivetrain Init Completed - Linear");
        opMode.telemetry.update();

    }

    public Drivetrain(OpMode opMode){

        this.iterative_OpMode = opMode;

        f = this.iterative_OpMode.hardwareMap.get(DcMotorEx.class, "f");
        r = this.iterative_OpMode.hardwareMap.get(DcMotorEx.class, "r");
        l = this.iterative_OpMode.hardwareMap.get(DcMotorEx.class, "l");
        b = this.iterative_OpMode.hardwareMap.get(DcMotorEx.class, "b");

        ffOdom = this.iterative_OpMode.hardwareMap.servo.get("ffOdom");
        sfOdom = this.iterative_OpMode.hardwareMap.servo.get("sfOdom");


        f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        r.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        l.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        f.setDirection(DcMotorSimple.Direction.FORWARD);
        r.setDirection(DcMotorSimple.Direction.REVERSE);
        l.setDirection(DcMotorSimple.Direction.REVERSE);
        b.setDirection(DcMotorSimple.Direction.FORWARD);

        for (LynxModule module : this.iterative_OpMode.hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        opMode.telemetry.addLine("Drivetrain Init Completed - Iterative");
        opMode.telemetry.update();

        ffOdom.setPosition(0);
        sfOdom.setPosition(0);
    }

 //================== Utility Methods ============================================

    public void setFfOdomPos(double pos){
        ffOdom.setPosition(pos);
    }

    public void setSfOdomPos(double pos){
        sfOdom.setPosition(pos);
    }

    public void setMotorBrake(boolean brake){
        if (brake){
            f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            l.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else {
            f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            l.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void setMotorPowers(double powerF, double powerR, double powerL, double powerB) {
        f.setPower(powerF);
        r.setPower(powerR);
        l.setPower(powerL);
        b.setPower(powerB);
    }
    public void setAllMotors(double power) {
        f.setPower(power);
        r.setPower(power);
        l.setPower(power);
        b.setPower(power);
    }

    public void turn(double power, boolean turnRight) {
        power *= turnRight ? 1 : -1;
        f.setPower(power);
        r.setPower(-power);
        l.setPower(power);
        b.setPower(-power);
    }

    public void moveForwardTime(double power, double time) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < time) {
            setAllMotors(power);
        }
        setAllMotors(0);
    }

    public int[] getEncoders(){
        return new int[]{
                f.getCurrentPosition(),
                r.getCurrentPosition(),
                -l.getCurrentPosition(), // TODO: FIGURE OUT WHY ITS REPORTING INCORRECTLY
                b.getCurrentPosition()
        };
    }

    public int[] getOdom(){
        return new int[]{
                f.getCurrentPosition(),
                l.getCurrentPosition()
        };
    }

    public double getEncodersLR(){
        double left = l.getCurrentPosition();
        double right = r.getCurrentPosition();
        double counter = 2;

        if (right == 0){counter--;}
        if (left == 0){counter--;}

        iterative_OpMode.telemetry.addData("left", l.getCurrentPosition());
        iterative_OpMode.telemetry.addData("right", r.getCurrentPosition());

        return (left + right) / counter;
    }

    public double getEncodersFB(){
        double front = f.getCurrentPosition();
        double back = b.getCurrentPosition();
        double counter = 2;

        if (front == 0){counter--;}
        if (back == 0){counter--;}

        iterative_OpMode.telemetry.addData("front", f.getCurrentPosition());
        iterative_OpMode.telemetry.addData("back", b.getCurrentPosition());
        return (front + back) / counter;
    }

    public double getEncodersAll(){
        double left = l.getCurrentPosition();
        double right = r.getCurrentPosition();
        double front = f.getCurrentPosition();
        double back = b.getCurrentPosition();
        double counter = 4;

        // encoder values of 0 indicate the encoder wire is unplugged
        if (front == 0){counter--;}
        if (back == 0) {counter--;}
        if (right == 0){counter--;}
        if (left == 0) {counter--;}

        iterative_OpMode.telemetry.addData("left", left);
        iterative_OpMode.telemetry.addData("right", right);
        iterative_OpMode.telemetry.addData("front", front);
        iterative_OpMode.telemetry.addData("back", back);

        return (front + left + right + back) / counter;
    }


//============================ Auto ===============================================
public void spinDuck(double turnPower, double movePower, double moveAngle, double angle, double time, boolean turnRight){
    double powerF, powerR, powerL, powerB, newX, newY;

    moveAngle *= -1; //correcting opposite thing
    moveAngle += (Math.PI / 6); //correcting spin angle change
    moveAngle += 0.25 * Math.cos((0.7 * Math.PI) * time);

    int turnDirection = -1; //cock
                            // what
    if (turnRight) {
        turnDirection = 1;
    }



    newX = rotateX( Math.cos(moveAngle), Math.sin(moveAngle), angle);
    newY = rotateY(Math.cos(moveAngle), Math.sin(moveAngle), angle);

    powerF = (movePower * newX) + (turnPower * turnDirection);
    powerR = (movePower * newY) + (turnPower * turnDirection);
    powerL = (movePower * newY) - (turnPower * turnDirection);
    powerB = (movePower * newX) - (turnPower * turnDirection);

    setMotorPowers(powerF, powerR, powerL, powerB);
    }


//========================= Tele-Op Methods =======================================

    public double lockHeadingAngle(double initAngle, double angle) {
        return Math.sin(angle - initAngle);
    }

    public double lockNearestX(double angle){

        if (angle < 0){
            if (angle < (-Math.PI / 2)){
                return (-3 * Math.PI) / 4;
            }
            else{
                return (-Math.PI / 4);
            }
        }
        else {
            if (angle > (Math.PI / 2)){
                return (3 * Math.PI) / 4;
            }
            else {
                return (Math.PI / 4);
            }
        }

    }

    public void moveTeleOp_Plus(double x, double y, double z, double speedMultiplier, double turnMultiplier) { // Lstick.x, Lstick.y, Rstick.x
        double powerF, powerR, powerL, powerB;
        powerF = (x * speedMultiplier) + (z * turnMultiplier); //front
        powerR = (y * speedMultiplier) + (z * turnMultiplier); // left
        powerL = (y * speedMultiplier) - (z * turnMultiplier); // right
        powerB = (x * speedMultiplier) - (z * turnMultiplier); // back
        double m = Math.abs(max(powerF, powerR, powerL, powerB));
        if (m > 1){
            powerF /= m;
            powerR /= m;
            powerL /= m;
            powerB /= m;
        }

        setMotorPowers(powerF, powerR, powerL, powerB);
    }
    
    private double max(double ... vals){
        double m = vals[0];
        for (double val: vals
             ) {
            m = Math.max(val, m);
        }
        return m;
    }

    private double sigmoid(double x){
        return 2.0 / (1 + Math.exp(-2.5 * x)) - 1;
    }

    public void moveTeleOp_X(double x, double y, double z, double speedMultiplier, double turnMultiplier){ // Lstick.x, Lstick.y, Rstick.x
        double powerF, powerR, powerL, powerB;
        powerF = ((x + y) * speedMultiplier) + (z * turnMultiplier); //front
        powerR = ((-x + y) * speedMultiplier) + (z * turnMultiplier); // left
        powerL = ((-x + y) * speedMultiplier) - (z * turnMultiplier); // right
        powerB = ((x + y) * speedMultiplier) - (z * turnMultiplier); // back

        double m = Math.abs(max(powerF, powerR, powerL, powerB));
        if (m > 1){
            powerF /= m;
            powerR /= m;
            powerL /= m;
            powerB /= m;
        }

        setMotorPowers(powerF, powerR, powerL, powerB);
    }

    public void moveGyroTeleOp_Plus(double x, double y, double z, double speedMultiplier, double turnMultiplier, double angle){ //think out with team
        //correspond l & r with shifted by pi / 2 mutiplied by the sign of cosine because changes direction when on the other side
        //turrent offset works by changing the angle offset rather than direct motor control
        double powerF, powerR, powerL, powerB, newX, newY;

        newX = rotateX(x,y,angle);
        newY = rotateY(x,y,angle);

        powerF = (newX * speedMultiplier) + (z * turnMultiplier); //front
        powerR = (newY * speedMultiplier) + (z * turnMultiplier); // left
        powerL = (newY * speedMultiplier) - (z * turnMultiplier); // right
        powerB = (newX * speedMultiplier) - (z * turnMultiplier); // back

        double m = Math.abs(max(powerF, powerR, powerL, powerB));
        if (m > 1){
            powerF /= m;
            powerR /= m;
            powerL /= m;
            powerB /= m;
        }

        setMotorPowers(powerF, powerR, powerL, powerB);
    }

    public void moveGyroTeleOp_X(double x, double y, double z, double speedMultiplier, double turnMultiplier, double angle){   //think out with team
        double powerF, powerR, powerL, powerB, newX, newY;

        newX = rotateX(x,y,angle);
        newY = rotateY(x,y,angle);

        powerF = ((newX + newY) * speedMultiplier) + (z * turnMultiplier); //front
        powerR = ((-newX + newY) * speedMultiplier) + (z * turnMultiplier); // left
        powerL = ((-newX + newY) * speedMultiplier) - (z * turnMultiplier); // right
        powerB = ((newX + newY) * speedMultiplier) - (z * turnMultiplier); // back

        double m = Math.abs(max(powerF, powerR, powerL, powerB));
        if (m > 1){
            powerF /= m;
            powerR /= m;
            powerL /= m;
            powerB /= m;
        }

        setMotorPowers(powerF, powerR, powerL, powerB);
    }



    public double rotateX(double x, double y, double angle){
        return (Math.cos(angle) * x) - (Math.sin(angle) * y);
    }

    public double rotateY(double x, double y, double angle){
        return (Math.sin(angle) * x) + (Math.cos(angle) * y);
    }


    public double rotateFBY(double x, double y, double angle){
        double output = (-Math.cos(angle + (Math.PI / 4)) * x) + (Math.sin(angle + (Math.PI / 4)) * y);
        return output;
    }

    public double rotateLRY(double x, double y, double angle){
        double output = (-Math.cos(angle + (Math.PI / 4)) * y) - (Math.sin(angle + (Math.PI / 4)) * x);
        return output;
    }

    public int getForwardEncoder(){
        return l.getCurrentPosition();
    }

    public int getSideEncoder(){
        return f.getCurrentPosition() * -1;
    }

    public void lowerOdom(){
        ffOdom.setPosition(FF_LOW);
        sfOdom.setPosition(SF_LOW);
    }

    public void raiseOdom(){
        ffOdom.setPosition(0);
        sfOdom.setPosition(0);
    }


    public void moveInches(double inches, double power, boolean strafe, double timeout){
        double powerF = 0, powerR = 0, powerL = 0, powerB = 0;
        double multiplier = -1;
        double initEncoder;
        ElapsedTime timer = new ElapsedTime();

        if (inches < 0){
            multiplier *= -1;
        }

        if (strafe){
            initEncoder = getSideEncoder();
            powerF = -power * multiplier;
            powerB = -power * multiplier;
        }
        else {
            initEncoder = getForwardEncoder();
            powerR = power * multiplier;
            powerL = power * multiplier;
        }

        double goalEncoder = (inches * ENCODER_IN_INCHES) + initEncoder;

        timer.reset();
        if (strafe){
            if (goalEncoder > initEncoder){

                while (goalEncoder > getSideEncoder() && timer.seconds() < timeout && linear_OpMode.opModeIsActive()){


                    linear_OpMode.telemetry.addData("goalEncoder", goalEncoder);
                    linear_OpMode.telemetry.addData("initEncoder", initEncoder);
                    linear_OpMode.telemetry.addData("sideEncoder", getSideEncoder());
                    linear_OpMode.telemetry.update();

                    setMotorPowers(powerF,powerR,powerL,powerB);


                }
            }
            else {
                while (goalEncoder < getSideEncoder() && timer.seconds() < timeout && linear_OpMode.opModeIsActive()){

                    linear_OpMode.telemetry.addData("goalEncoder", goalEncoder);
                    linear_OpMode.telemetry.addData("initEncoder", initEncoder);
                    linear_OpMode.telemetry.addData("sideEncoder", getSideEncoder());
                    linear_OpMode.telemetry.update();

                    setMotorPowers(powerF,powerR,powerL,powerB);


                }
            }
        }
        else{
            if (goalEncoder > initEncoder){
                while (goalEncoder > getForwardEncoder() && timer.seconds() < timeout && linear_OpMode.opModeIsActive()){

                    linear_OpMode.telemetry.addData("goalEncoder", goalEncoder);
                    linear_OpMode.telemetry.addData("initEncoder", initEncoder);
                    linear_OpMode.telemetry.addData("forwardEncoder", getForwardEncoder());
                    linear_OpMode.telemetry.update();
                    setMotorPowers(powerF,powerR,powerL,powerB);


                    //linear_OpMode.telemetry.addData("", goalEncoder);
                }
            }
            else {
                while (goalEncoder < getForwardEncoder() && timer.seconds() < timeout && linear_OpMode.opModeIsActive()){
                    linear_OpMode.telemetry.addData("goalEncoder", goalEncoder);
                    linear_OpMode.telemetry.addData("initEncoder", initEncoder);
                    linear_OpMode.telemetry.addData("forwardEncoder", getForwardEncoder());
                    linear_OpMode.telemetry.update();
                    setMotorPowers(powerF,powerR,powerL,powerB);

                }
            }
        }

        setMotorPowers(0,0,0,0);
    }

    public void runToEncoderPositions(int f, int r, int l, int b){
        PID pidf = new PID(mPID.kP, mPID.kI, mPID.kD, f),
            pidr = new PID(mPID.kP, mPID.kI, mPID.kD, r),
            pidl = new PID(mPID.kP, mPID.kI, mPID.kD, l),
            pidb = new PID(mPID.kP, mPID.kI, mPID.kD, b);

        ElapsedTime timer = new ElapsedTime();

        int[] encoders = getEncoders();
        while (Math.abs(f - encoders[0]) +
                Math.abs(r - encoders[1]) +
                Math.abs(l - encoders[2]) +
                Math.abs(b - encoders[3])
                > 5
        ){
            setMotorPowers(
                    pidf.loop(encoders[0], timer.seconds()),
                    pidr.loop(encoders[1], timer.seconds()),
                    pidl.loop(encoders[2], timer.seconds()),
                    pidb.loop(encoders[3], timer.seconds())
            );

            TelemetryPacket p = new TelemetryPacket();
            encoders = getEncoders();
            linear_OpMode.telemetry.addData("f", encoders[0]);
            linear_OpMode.telemetry.addData("r", encoders[1]);
            linear_OpMode.telemetry.addData("l", encoders[2]);
            linear_OpMode.telemetry.addData("b", encoders[3]);
            p.put("f", encoders[0]);
            p.put("r", encoders[1]);
            p.put("l", encoders[2]);
            p.put("b", encoders[3]);
            dashboard.sendTelemetryPacket(p);
        }
    }

    public void runToEncoderPositionsOdom(int f, int l){
        PID
                pidf = new PID(0.1, 0, 0, f),
                pidr = new PID(0.1, 0, 0, l),
                pidl = new PID(0.1, 0, 0, l),
                pidb = new PID(0.1, 0, 0, f);

        ElapsedTime timer = new ElapsedTime();

        int[] encoders = getOdom();
        while (Math.abs(f - encoders[0]) +
                Math.abs(l - encoders[1]) +
                Math.abs(l - encoders[1]) +
                Math.abs(f - encoders[0])
                > 5
                && linear_OpMode.opModeIsActive()
        ){
            setMotorPowers(
                    pidf.loop(encoders[0], timer.seconds()),
                    pidl.loop(encoders[1], timer.seconds()),
                    pidr.loop(encoders[1], timer.seconds()),
                    pidb.loop(encoders[0], timer.seconds())
            );

            linear_OpMode.telemetry.addData("ff", encoders[0]);
            linear_OpMode.telemetry.addData("sf", encoders[1]);
            linear_OpMode.telemetry.update();

            encoders = getEncoders();
        }
    }

    public void turnPID(double newAngle){
        Sensors sensor = new Sensors(linear_OpMode);
        double initHeading = sensor.getFirstAngle();

        PID pid = new PID(0.3, 0, 0, newAngle);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (Math.abs(newAngle - sensor.getFirstAngle()) > 5){
            setAllMotors(pid.loop(sensor.getFirstAngle(), timer.seconds()));
        }
    }
    public void turnToPID(double newAngle, Sensors sensor, double timeout){
        // Sensors sensor = new Sensors(linear_OpMode);
        // double initHeading = startHeading;

        PID pid = new PID(spinPID.kP, spinPID.kI, spinPID.kD, newAngle);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (Math.abs(sensor.getFirstAngle() - newAngle) > turnError * Math.PI){
            double newPower = pid.loop(sensor.getFirstAngle(), timer.seconds());
            setMotorPowers(-newPower, -newPower, newPower, newPower);
            linear_OpMode.telemetry.addData("motor powers", newPower);
            linear_OpMode.telemetry.addData("current rotation", sensor.getFirstAngle() / Math.PI * 180);
            linear_OpMode.telemetry.update();

            if (timer.seconds() >= timeout)
                break;
        }
    }

    public FtcDashboard getDashboard() {
        return dashboard;
    }

    public void setDashboard(FtcDashboard dashboard) {
        this.dashboard = dashboard;
    }
}
