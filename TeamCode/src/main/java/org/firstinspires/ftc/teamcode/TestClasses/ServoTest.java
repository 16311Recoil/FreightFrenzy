package org.firstinspires.ftc.teamcode.TestClasses;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "ServoTest", group = "Controlled")
//@Disabled
public class ServoTest extends LinearOpMode {

    private Servo armRotater;
    private boolean changeDpadUp = false;
    private boolean changeDpadDown = false;
    private boolean changeY;
    private double pos1 = 0;
    private double pos2 = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        armRotater = hardwareMap.servo.get("clawRotator");
        armRotater.setDirection(Servo.Direction.FORWARD);
        //armRotater.setDirection(Servo.Direction.REVERSE);
        armRotater.setPosition(pos1);
        boolean statePos2 = false;
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a){
                armRotater.setPosition(pos1);
            }
            if (gamepad1.b){
                armRotater.setPosition(pos2);
            }

            if (gamepad1.y && !changeY){
                statePos2 = !statePos2;
            }

            if (gamepad1.dpad_down && !changeDpadDown){
                if (statePos2){
                    pos2 -= .01;
                }
                else {
                    pos1 -= .01;
                }

            }
            if (gamepad1.dpad_up && !changeDpadUp){
                if (statePos2){
                    pos2 += .01;
                }
                else {
                    pos1 += .01;
                }
            }

            telemetry.addData("Pos2State", statePos2);
            telemetry.addData("pos1", pos1);
            telemetry.addData("pos2", pos2);
            changeDpadDown = gamepad1.dpad_down;
            changeDpadUp = gamepad1.dpad_up;
            changeY = gamepad1.y;
            telemetry.update();
        }






    }
}