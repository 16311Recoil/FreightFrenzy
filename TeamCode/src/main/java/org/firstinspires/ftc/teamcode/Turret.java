package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.apache.commons.math3.geometry.euclidean.twod.Line;

public class Turret {
    private LinearOpMode linear_OpMode;
    private OpMode iterative_OpMode;

    DcMotor turretMotor;

    private int targetAngle = 0;

    private final double ROTATION_SPEED = 0;

    public Turret(LinearOpMode opMode){
        linear_OpMode = opMode;

        turretMotor = opMode.hardwareMap.get(DcMotor.class, "turret_motor");
        turretMotor.setPower(0);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        opMode.telemetry.addLine("Turret Init Completed - Linear");
        opMode.telemetry.update();
    }

    public Turret(OpMode opMode){
        iterative_OpMode = opMode;

        turretMotor = opMode.hardwareMap.get(DcMotor.class, "turret_motor");
        turretMotor.setPower(0);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        opMode.telemetry.addLine("Turret Init Completed - Iterative");
        opMode.telemetry.update();
    }

    public void setDirection(int angle){
        targetAngle = angle;
    }

    public void update(double robotDirection){
        turretMotor.setTargetPosition(nearestAngleToTurret((int)(targetAngle - robotDirection)));
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
}
