package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drivetrain;

@Autonomous(name="BlueAutoPathed", group="Auto")
public class BlueAutoPathed extends LinearOpMode {
    // Target encoder values at each step, format: F, R, L, B
    // TODO: Measure pathing for Blue side
    int[][][] paths = {
            {
                    {0, 0, 0, 0},
                    {0, 0, 0, 0},
                    {0, 0, 0, 0}
            },
            {
                    {0, 0, 0, 0},
                    {0, 0, 0, 0},
                    {0, 0, 0, 0}
            }
    };
    // TODO: Find spin time
    int spinTime = 1500; // ms
    int spinPath = 0; // Path after which the spinning happens

    double power = 0.3, spinPower = 0.5;
    Drivetrain drivetrain;

    @Override
    public void runOpMode(){
        drivetrain = new Drivetrain(this);

        waitForStart();

        for (int i = 0; i < paths.length; i ++) {
            for (int[] seq: paths[i]
                ) {

                int[] encoders = drivetrain.getEncoders();
                int sumError = 0;
                for (int j = 0; j < 4; j ++)
                    sumError += Math.abs(seq[i] - encoders[i]);

                while (sumError > 10)
                {
                    encoders = drivetrain.getEncoders();
                    drivetrain.setMotorPowers(
                            Math.signum(seq[0] - encoders[0]) * power,
                            Math.signum(seq[1] - encoders[1]) * power,
                            Math.signum(seq[2] - encoders[2]) * power,
                            Math.signum(seq[3] - encoders[3]) * power
                    );

                    for (int j = 0; j < 4; j ++)
                        sumError += Math.abs(seq[i] - encoders[i]);
                }
            }
            if (i == spinPath){
                ElapsedTime timer =  new ElapsedTime();
                timer.reset();
                while (timer.milliseconds() < spinTime);
                    drivetrain.spinDuck(spinPower, 0.2, Math.PI * 1.75, Math.PI * 0.5, false);
            }
        }

    }
}
