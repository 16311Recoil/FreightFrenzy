package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drivetrain;

@TeleOp(name="Measuring", group="utility")
public class Measuring extends OpMode {
    Drivetrain drivetrain;

    @Override
    public void init(){
        drivetrain = new Drivetrain(this);
    }

    @Override
    public void loop(){
        int[] vals = drivetrain.getEncoders();
        telemetry.addData("F", vals[0]);
        telemetry.addData("R", vals[1]);
        telemetry.addData("L", vals[2]);
        telemetry.addData("B", vals[3]);
        telemetry.update();
    }
}
