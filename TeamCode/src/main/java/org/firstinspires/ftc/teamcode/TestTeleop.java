package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "testTeleop", group = "teleop")
public class TestTeleop extends OpMode{
    DcMotor f;
    DcMotor r;
    DcMotor l;
    DcMotor b;
    @Override
    public void init() {
        f = hardwareMap.get(DcMotor.class, "f");
        r = hardwareMap.get(DcMotor.class, "r");
        l = hardwareMap.get(DcMotor.class, "l");
        b = hardwareMap.get(DcMotor.class, "b");

    }

    @Override
    public void loop() {
        moveTeleOp_Plus(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 1, 1);
    }

    public void moveTeleOp_Plus(double x, double y, double z, double speedMultiplier, double turnMultiplier){ // Lstick.x, Lstick.y, Rstick.x
        double powerF, powerR, powerL, powerB;
        powerF = Range.clip((x * speedMultiplier) + (z * turnMultiplier), -1,1); //front
        powerR = Range.clip((y * speedMultiplier) - (z * turnMultiplier), -1,1); // left
        powerL = Range.clip((y * speedMultiplier) + (z * turnMultiplier), -1,1); // right
        powerB = Range.clip((x * speedMultiplier) - (z * turnMultiplier), -1,1); // back

        setMotorPowers(powerF, powerR, powerL, powerB);
    }
    public void setMotorPowers(double powerF, double powerR, double powerL, double powerB) {
        f.setPower(powerF);
        r.setPower(powerR);
        l.setPower(powerL);
        b.setPower(powerB);
    }

}
