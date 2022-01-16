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

        robot.getManip().setArmRotatorPower(0.3);
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

        // TODO: Fix vision
        if (pos == VisionTestRed.DeterminationPipeline.MarkerPosition.LEFT)
            hub_pos = 50;
        else if (pos == VisionTestRed.DeterminationPipeline.MarkerPosition.CENTER)
            hub_pos = 105;
        else{
            hub_pos = 220;
            extra++;
        }

        // raise arm BEFORE we move forward
        robot.getManip().goToPosition(hub_pos);
        robot.getManip().rotateClawUp();

        // move towards the hub
        // TODO: Move correct distance
        robot.getDrivetrain().moveInches(13.5 + extra, power, false, 4);
        Thread.sleep(2000);

        // drop block
        robot.getManip().mechRelease();
        Thread.sleep(2000);

        // Go back
        robot.getDrivetrain().moveInches(-13.5 - extra, power, false, 4);
        robot.getManip().rotateClawDown();

        Thread.sleep(2000);
        robot.getManip().goToPosition(80);

        // Move to duck
        robot.getDrivetrain().moveInches(4 + extra, power, false, 5);
        robot.getDrivetrain().moveInches(-36 - extra, power, true, 7);
        Thread.sleep(1000);

        // Put arm into excalibur mode
        robot.getManip().setArmRotatorPower(0.5);
        for (int i = 160; i <= 360; i += 100)
            robot.getManip().goToPosition(i);
        Thread.sleep(2000);

        double init_heading = robot.getSensors().getFirstAngle();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.milliseconds() < 7000){
            robot.getDrivetrain().spinDuck(0.3, 0.1, 1.25 * Math.PI, robot.getSensors().getFirstAngle() - init_heading, 4, false);
        }
        robot.getDrivetrain().setAllMotors(0);

        // move away from wall to allow for spin
        double curAngle = robot.getSensors().getFirstAngle();
        double range = Math.sin(Math.PI * 3 / 4) - Math.cos(Math.PI * 3 / 4);

        // TODO: Test that this math is correct

        double moveForward = (Math.cos(curAngle) - Math.sin(curAngle)) / range * 10,
        moveSide = (Math.sin(curAngle) + Math.cos(curAngle)) / range * 10;

        TelemetryPacket p = new TelemetryPacket();
        p.put("moveForward", moveForward);
        p.put("moveSide", moveSide);
        p.put("currentAngle", curAngle / Math.PI * 180);
        dashboard.sendTelemetryPacket(p);

        robot.getDrivetrain().moveInches(moveForward, power, false, 2);
        robot.getDrivetrain().moveInches(moveSide, power, true, 2);

        Thread.sleep(1000);

        // Adjust rotation back to properly use moveInches
        robot.getDrivetrain().turnToPID(0, robot.getSensors(), 4);

        // Back into the wall to correct any rotation imperfections
        robot.getDrivetrain().moveInches(-30 - extra, power, false, 4);
        Thread.sleep(2000);

        robot.getManip().setArmRotatorPower(0.1);
        for (int i = 350; i >= 50; i -= 100)
            robot.getManip().goToPosition(i);
        // Park in freight area
        robot.getDrivetrain().moveInches(150 + extra, power, true, 7);
        Thread.sleep(2000);

        // Setup for teleop
        robot.getManip().rotateClawDown();
        robot.getManip().mechRelease();
        Thread.sleep(2000);
    }
}
