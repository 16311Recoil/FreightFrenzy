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
        armRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                armRotator.setTargetPosition(pos1);
            }
            else {
                armRotator.setTargetPosition(pos2);
            }

            if (gamepad1.dpad_up){
                pos2 ++;
            }
            if (gamepad1.dpad_down){
                pos2 --;
            }
            telemetry.addData("target position", armRotator.getTargetPosition());
            telemetry.addData("current position", armRotator.getCurrentPosition());
            telemetry.addData("pos2", pos2);
            telemetry.update();
            armRotator.setPower(1);
        }
    }
}