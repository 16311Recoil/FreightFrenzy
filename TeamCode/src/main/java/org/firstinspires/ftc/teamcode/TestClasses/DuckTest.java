package org.firstinspires.ftc.teamcode.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.DrivetrainRR;
import org.firstinspires.ftc.teamcode.Sensors;

@Autonomous(name = "DuckTest", group = "Controlled")
public class DuckTest extends LinearOpMode {

    private Drivetrain dr;
    private Sensors sensor;
    private double init_Heading;
    private ElapsedTime timer;


    @Override
    public void runOpMode() throws InterruptedException {


        dr = new Drivetrain(this);
        sensor = new Sensors(this);
        init_Heading = sensor.getFirstAngle();
        timer = new ElapsedTime();


        waitForStart();

        dr.setMotorBrake(false);
        timer.reset();
        while(timer.seconds() < 5){
            dr.spinDuck(0.5, .3, (Math.PI) / 2, sensor.getFirstAngle() - init_Heading, true);
        }


        //dr.followTrajectory();
    }
}
