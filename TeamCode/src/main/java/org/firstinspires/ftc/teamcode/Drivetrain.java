package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Drivetrain {

    private DcMotorEx f, r, l, b;
    private LinearOpMode linear_OpMode;
    private OpMode interative_OpMode;

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

        opMode.telemetry.addLine("Drivetrain Init Completed - Linear");
        opMode.telemetry.update();

    }

    public Drivetrain(OpMode opMode){

        this.interative_OpMode = opMode;

        f = this.interative_OpMode.hardwareMap.get(DcMotorEx.class, "f");
        r = this.interative_OpMode.hardwareMap.get(DcMotorEx.class, "r");
        l = this.interative_OpMode.hardwareMap.get(DcMotorEx.class, "l");
        b = this.interative_OpMode.hardwareMap.get(DcMotorEx.class, "b");

        f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        l.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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




}
