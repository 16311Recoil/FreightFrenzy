package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Crab;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Manipulator;
import org.firstinspires.ftc.teamcode.PID;
import org.firstinspires.ftc.teamcode.VisionTest;
import org.firstinspires.ftc.teamcode.VisionTestRed;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name="RedAutoPathed", group="Auto")
public class RedAutoPathed extends LinearOpMode {
    Crab robot;
    VisionTestRed.DeterminationPipeline pipeline;
    FtcDashboard dashboard;
    VisionTestRed.DeterminationPipeline.MarkerPosition pos;
    public static int extra = -4;
    public static double power = 0.3;

    @Override
    public void runOpMode() throws InterruptedException{
        robot = new Crab(this);

        robot.getDrivetrain().lowerOdom();
        robot.getManip().rotateClawUp();
        robot.getManip().mechGrab();
        dashboard = FtcDashboard.getInstance();

        pipeline = new VisionTestRed.DeterminationPipeline();
        robot.getSensors().getWebcam().setPipeline(pipeline);
        robot.getSensors().getWebcam().openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robot.getSensors().getWebcam().startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        while (!isStarted()) {
            TelemetryPacket p = new TelemetryPacket();
            dashboard.startCameraStream(robot.getSensors().getWebcam(), 30);

            telemetry.addData("pos", pipeline.getAnalysis());
            p.put("pos", pipeline.getAnalysis());
            dashboard.sendTelemetryPacket(p);

            pos = pipeline.getAnalysis();
        }


        waitForStart();

        robot.getTurret().setPosition(-83);


        // robot.getManip().mechGrab();

        int hub_pos;

        if (pos == VisionTestRed.DeterminationPipeline.MarkerPosition.LEFT)
            hub_pos = 50;
        else if (pos == VisionTestRed.DeterminationPipeline.MarkerPosition.CENTER)
            hub_pos = 105;
        else{
            hub_pos = 220;
            extra++;
        }


/*
        // TODO: Uncomment manip code after adjusting values in manip class (check manip TODOs)

        // raise arm BEFORE we move forward
        //robot.getManip().placePresetLevel(hub_pos);
        robot.getManip().goToPosition(hub_pos);
        // manipulator.rotateClawUp();

        // move towards the hub
        robot.getDrivetrain().moveInches(13.5 + extra, power, false, 4);
        Thread.sleep(2000);

        // drop block
        robot.getManip().mechRelease();
        Thread.sleep(2000);
        // drop block
        // manipulator.mechRelease();

        // go back

        robot.getDrivetrain().moveInches(-13.5 - extra, power, false, 4);

        Thread.sleep(2000);
        robot.getManip().goToPosition(80);
*/
        // TODO: Go to duck
        robot.getDrivetrain().moveInches(4 + extra, power, false, 5);
        robot.getDrivetrain().moveInches(-36 - extra, power, true, 7);
        Thread.sleep(1000);

        // TODO: Get duck

        robot.getManip().goToPosition(350);
        double init_heading = robot.getSensors().getFirstAngle();
        ElapsedTime timer = new ElapsedTime();
        telemetry.addData("init heading", init_heading);
        telemetry.update();
        timer.reset();
        while (timer.milliseconds() < 3000){
            robot.getDrivetrain().spinDuck(0.3, 0.1, 1.25 * Math.PI, robot.getSensors().getFirstAngle() - init_heading, 4, false);
        }
        Thread.sleep(6000);

        // TODO: Adjust back

        telemetry.addData("starting at position", robot.getSensors().getFirstAngle());
        while (Math.abs(robot.getSensors().getFirstAngle()) > 0.02){
            if (robot.getSensors().getFirstAngle() > 0){
                robot.getDrivetrain().setAllMotors(-0.3);
            }
            else {
                robot.getDrivetrain().setAllMotors(0.3);
            }
        }

        telemetry.addData("ended at position", robot.getSensors().getFirstAngle());
        telemetry.update();
        Thread.sleep(2000);

        robot.getDrivetrain().moveInches(10 + extra, power, true, 4);
        robot.getDrivetrain().moveInches(-10 - extra, power, false, 4);
        Thread.sleep(2000);

        // park in freight area
        robot.getDrivetrain().moveInches(76, power, true, 5);
        Thread.sleep(2000);

        robot.getManip().rotateClawDown();
        robot.getManip().mechRelease();
        Thread.sleep(2000);
    }
}
