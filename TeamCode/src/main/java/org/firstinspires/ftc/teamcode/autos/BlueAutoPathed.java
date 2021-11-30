package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.PID;

@Autonomous(name="BlueAutoPathed", group="Auto")
public class BlueAutoPathed extends LinearOpMode {
    Drivetrain drivetrain;

    @Override
    public void runOpMode(){
        drivetrain = new Drivetrain(this);
        drivetrain.lowerOdom();

        waitForStart();

        // -46184, 7908
        // TODO: Make the actual path
        drivetrain.runToEncoderPositionsOdom(-46184, 7908);
    }
}
