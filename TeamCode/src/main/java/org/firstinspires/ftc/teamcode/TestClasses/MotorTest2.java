package org.firstinspires.ftc.teamcode.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "MotorTest2", group = "Controlled")
public class MotorTest2 extends LinearOpMode {
    private DcMotor motor;
    private int pos1 = 0, pos2 = 180;

    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.get(DcMotorEx.class, "motor");


        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a){
                motor.setPower(0.3);
            }
        }
    }
}