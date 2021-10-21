package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Drivetrain{

    private DcMotorEx f, r, l, b;
    private LinearOpMode linear_OpMode;
    private OpMode iterative_OpMode;
    private FtcDashboard dashboard;

    public Drivetrain(LinearOpMode opMode){

        this.linear_OpMode = opMode;

        f = this.linear_OpMode.hardwareMap.get(DcMotorEx.class, "f");
        r = this.linear_OpMode.hardwareMap.get(DcMotorEx.class, "r");
        l = this.linear_OpMode.hardwareMap.get(DcMotorEx.class, "l");
        b = this.linear_OpMode.hardwareMap.get(DcMotorEx.class, "b");

        f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        l.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        f.setDirection(DcMotorSimple.Direction.FORWARD);
        r.setDirection(DcMotorSimple.Direction.FORWARD);
        l.setDirection(DcMotorSimple.Direction.REVERSE);
        b.setDirection(DcMotorSimple.Direction.FORWARD);

        for (LynxModule module : this.linear_OpMode.hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        opMode.telemetry.addLine("Drivetrain Init Completed - Linear");
        opMode.telemetry.update();

    }

    public Drivetrain(OpMode opMode){

        this.iterative_OpMode = opMode;

        f = this.iterative_OpMode.hardwareMap.get(DcMotorEx.class, "f");
        r = this.iterative_OpMode.hardwareMap.get(DcMotorEx.class, "r");
        l = this.iterative_OpMode.hardwareMap.get(DcMotorEx.class, "l");
        b = this.iterative_OpMode.hardwareMap.get(DcMotorEx.class, "b");

        f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        l.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        f.setDirection(DcMotorSimple.Direction.FORWARD);
        r.setDirection(DcMotorSimple.Direction.FORWARD);
        l.setDirection(DcMotorSimple.Direction.REVERSE);
        b.setDirection(DcMotorSimple.Direction.FORWARD);

        for (LynxModule module : this.iterative_OpMode.hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        opMode.telemetry.addLine("Drivetrain Init Completed - Iterative");
        opMode.telemetry.update();

    }

 //================== Utility Methods ============================================

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
        r.setPower(power);
    }

    public void turn(double power, boolean turnRight) {
        int turnMultiplier = -1;
        if (turnRight) {
            turnMultiplier = 1;
        }
        power *= turnMultiplier;
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

        if (front == 0){counter--;}
        if (back == 0) {counter--;}
        if (right == 0){counter--;}
        if (left == 0) {counter--;}

        iterative_OpMode.telemetry.addData("left", l.getCurrentPosition());
        iterative_OpMode.telemetry.addData("right", r.getCurrentPosition());
        iterative_OpMode.telemetry.addData("front", f.getCurrentPosition());
        iterative_OpMode.telemetry.addData("back", b.getCurrentPosition());

        return (front + left + right + back) / counter;
    }


//============================ Auto ===============================================
    public void spinDuck(){
        double spinAmount = 2000; //test
        double initEncoders = getEncodersAll();
        while (spinAmount > initEncoders - getEncodersAll()){
            turn(0.3, false);
        }
        setAllMotors(0);
    }


//========================= Tele-Op Methods =======================================

    public double lockHeadingAngle(double initAngle, double angle){
        return Math.sin(angle - initAngle);
    }

    public void moveTeleOp_Plus(double x, double y, double z, double speedMultiplier, double turnMultiplier){ // Lstick.x, Lstick.y, Rstick.x
        double powerF, powerR, powerL, powerB;
        powerF = Range.clip((x * speedMultiplier) + (z * turnMultiplier), -1,1); //front
        powerR = Range.clip((y * speedMultiplier) + (z * turnMultiplier), -1,1); // left
        powerL = Range.clip((y * speedMultiplier) - (z * turnMultiplier), -1,1); // right
        powerB = Range.clip((x * speedMultiplier) - (z * turnMultiplier), -1,1); // back

        setMotorPowers(powerF, powerR, powerL, powerB);
    }

    public void moveTeleOp_X(double x, double y, double z, double speedMultiplier, double turnMultiplier){ // Lstick.x, Lstick.y, Rstick.x
        double powerF, powerR, powerL, powerB;
        powerF = Range.clip(((x + y) * speedMultiplier) + (z * turnMultiplier), -1,1); //front
        powerR = Range.clip(((-x + y) * speedMultiplier) + (z * turnMultiplier), -1,1); // left
        powerL = Range.clip(((-x + y) * speedMultiplier) - (z * turnMultiplier), -1,1); // right
        powerB = Range.clip(((x + y) * speedMultiplier) - (z * turnMultiplier), -1,1); // back

        setMotorPowers(powerF, powerR, powerL, powerB);
    }

    public void moveGyroTeleOp_Plus(double x, double y, double z, double speedMultiplier, double turnMultiplier, double angle){ //think out with team
        //correspond l & r with shifted by pi / 2 mutiplied by the sign of cosine because changes direction when on the other side
        //turrent offset works by changing the angle offset rather than direct motor control
        double powerF, powerR, powerL, powerB, newX, newY;

        newX = rotateX(x,y,angle);
        newY = rotateY(x,y,angle);

        powerF = Range.clip((newX * speedMultiplier) + (z * turnMultiplier), -1,1); //front
        powerR = Range.clip((newY * speedMultiplier) + (z * turnMultiplier), -1,1); // left
        powerL = Range.clip((newY * speedMultiplier) - (z * turnMultiplier), -1,1); // right
        powerB = Range.clip((newX * speedMultiplier) - (z * turnMultiplier), -1,1); // back

        setMotorPowers(powerF, powerR, powerL, powerB);
    }

    public void moveGyroTeleOp_X(double x, double y, double z, double speedMultiplier, double turnMultiplier, double angle){   //think out with team
        double powerF, powerR, powerL, powerB, newX, newY;

        newX = rotateX(x,y,angle);
        newY = rotateY(x,y,angle);

        powerF = Range.clip(((newX + newY) * speedMultiplier) + (z * turnMultiplier), -1,1); //front
        powerR = Range.clip(((-newX + newY) * speedMultiplier) + (z * turnMultiplier), -1,1); // left
        powerL = Range.clip(((-newX + newY) * speedMultiplier) - (z * turnMultiplier), -1,1); // right
        powerB = Range.clip(((newX + newY) * speedMultiplier) - (z * turnMultiplier), -1,1); // back

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

    public FtcDashboard getDashboard() {
        return dashboard;
    }

    public void setDashboard(FtcDashboard dashboard) {
        this.dashboard = dashboard;
    }
}
