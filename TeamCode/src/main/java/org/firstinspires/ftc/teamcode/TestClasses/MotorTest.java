package org.firstinspires.ftc.teamcode.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MotorTest", group = "Controlled")
public class MotorTest extends OpMode {
    private DcMotor armRotator;
    private int pos1 = 0, pos2 = 20;
    private boolean changeDpadUp, changeDpadDown = false;

    public void init(){
        armRotator = this.hardwareMap.dcMotor.get("turret_motor");
        armRotator.setDirection(DcMotor.Direction.FORWARD);
        armRotator.setTargetPosition(pos1);
        armRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotator.setPower(1);
    }

    public void loop(){
        if (gamepad1.a) {
            armRotator.setTargetPosition(pos1);
        }
        if (gamepad1.b){
            armRotator.setTargetPosition(pos2);
        }

        if (gamepad1.dpad_up && !changeDpadUp){
            pos2 += 10;
        }
        changeDpadUp = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !changeDpadDown){
            pos2 -= 10;
        }
        changeDpadDown = gamepad1.dpad_down;


        telemetry.addData("target position", armRotator.getTargetPosition());
        telemetry.addData("current position", armRotator.getCurrentPosition());
        telemetry.addData("pos2", pos2);
        telemetry.update();
    }


}