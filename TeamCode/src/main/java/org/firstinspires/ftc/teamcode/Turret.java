package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.euclidean.twod.Line;

import java.util.concurrent.BrokenBarrierException;

public class Turret {
    private LinearOpMode linear_OpMode;
    private OpMode iterative_OpMode;

    DcMotor turretMotor;

    private int targetAngle = 0;

    private final double ROTATION_SPEED = 0;
    double ENCODER_TO_ANGLE_RATIO = 0.4643;

    //teleo-op utility
    private ElapsedTime timer = new ElapsedTime();
    private final int TOP_BOUND = 570;
    private final int LOW_BOUND = 0;
    private double goalEncoder = 0;
    private boolean stickPressed = false;
    private double prevTime = 0;

    public Turret(LinearOpMode opMode){
        linear_OpMode = opMode;

        turretMotor = opMode.hardwareMap.get(DcMotor.class, "turret_motor");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.7);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        opMode.telemetry.addLine("Turret Init Completed - Linear");
        opMode.telemetry.update();
    }

    public Turret(OpMode opMode){
        iterative_OpMode = opMode;

        turretMotor = opMode.hardwareMap.get(DcMotor.class, "turret_motor");

        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.7);

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
        turretMotor.setTargetPosition(nearestAngleToTurret((int)(targetAngle - robotDirection)));
    }

    public void setAngle(double angle){

        turretMotor.setTargetPosition((int)Math.round(angle / ENCODER_TO_ANGLE_RATIO));
    }

    public int nearestAngleToTurret(int angle){
        int turretAngle = turretMotor.getCurrentPosition();
        int normalizedAngle = normalizeAngle(angle), normalizedTurret = normalizeAngle(turretAngle);
        int invert = 1;
        if ((normalizedAngle < 90 && normalizedTurret > 270) || (normalizedTurret < 90 && normalizedAngle > 270))
            invert = -1;

        int change = Math.abs(normalizedAngle - normalizedTurret);
        change = change > 180 ? 360 - change : change;

        return turretAngle + change * (int)Math.signum(normalizedTurret - normalizedAngle) * invert;
    }

    public int normalizeAngle(int angle){
        return ((angle % 360) + 360) % 360;
    }

    public int getPosition(){
        return turretMotor.getCurrentPosition();
    }

    public void teleOpControls(double gamepad){
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
        //turretMotor.setTargetPosition((int)goalEncoder);
        movePID((int)goalEncoder);
        iterative_OpMode.telemetry.addData("timer", timer.seconds());
        iterative_OpMode.telemetry.addData("goal", goalEncoder);
        iterative_OpMode.telemetry.addData("prev Timer", prevTime);
        iterative_OpMode.telemetry.addData("stickPressed", stickPressed);
    }

    public void movePID(int encoderTarget){
        PID pid = new PID();
        ElapsedTime timer = new ElapsedTime();
        pid.setConstants(0.0065, 0,0.00002, encoderTarget);
        if (Math.abs(encoderTarget - getPosition()) > 5){
            turretMotor.setPower(pid.loop(turretMotor.getCurrentPosition(), timer.seconds()));
        }
    }

    public void movePIDLoop(int encoderTarget){
        PID pid = new PID();
        ElapsedTime timer = new ElapsedTime();
        pid.setConstants(0.3, 0,0, encoderTarget);
        while (Math.abs(encoderTarget - getPosition()) > 5){
            turretMotor.setPower(pid.loop(turretMotor.getCurrentPosition(), timer.seconds()));
        }
        turretMotor.setPower(0);
    }
}
