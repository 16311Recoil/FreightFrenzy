package org.firstinspires.ftc.teamcode.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Manipulator;

@TeleOp(name="ManipTeleop", group="teleop")
public class ManipTeleop extends OpMode {
    Manipulator manip;
    int manipHeight = 6;

    @Override
    public void init(){
        manip = new Manipulator(this);
    }

    @Override
    public void loop(){
        if (gamepad1.dpad_up){
            manipHeight += 0.01;
        }
        else if (gamepad1.dpad_down){
            manipHeight -= 0.01;
        }
        else{
            manipHeight += gamepad1.left_stick_y * 0.01;
        }

        manip.placeLevel(manipHeight);

        if (gamepad1.a)
            manip.magGrab();
        else
            manip.magRelease();

        if (gamepad1.b)
            manip.mechGrab();
        else
            manip.mechRelease();

        telemetry.addData("Target height", manipHeight);
        telemetry.update();
    }

}
