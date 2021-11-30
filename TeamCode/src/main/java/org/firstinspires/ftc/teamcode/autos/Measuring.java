package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drivetrain;

@TeleOp(name="Measuring", group="utility")
public class Measuring extends LinearOpMode {
    Drivetrain drivetrain;
    int c = 0;

    @Override
    public void runOpMode(){
        drivetrain = new Drivetrain(this);
        drivetrain.lowerOdom();

        waitForStart();

        while (!isStopRequested()) {
            int[] vals = drivetrain.getEncoders();
            telemetry.addData("FF", vals[0]);
            telemetry.addData("SF", vals[2]);
            telemetry.addData("random int", c);
            telemetry.update();
            c++;
        }
    }

}
