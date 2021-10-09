package org.firstinspires.ftc.teamcode.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MotorTest", group = "Controlled")
public class MotorTest extends LinearOpMode {
    private DcMotor armRotator;
    private int pos1 = 0, pos2 = 180;

    @Override
    public void runOpMode() throws InterruptedException {
        armRotator = hardwareMap.dcMotor.get("3");
        armRotator.setDirection(DcMotor.Direction.FORWARD);
        armRotator.setTargetPosition(pos1);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                armRotator.setTargetPosition(pos1);
            }
            else {
                armRotator.setTargetPosition(pos2);
            }
        }
    }
}
