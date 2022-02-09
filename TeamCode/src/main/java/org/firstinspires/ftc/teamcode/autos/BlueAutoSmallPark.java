package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Crab;
import org.firstinspires.ftc.teamcode.VisionTest;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="BlueAutoSmallPark", group="Auto")
public class BlueAutoSmallPark extends LinearOpMode {
    Crab robot;
    VisionTest.DeterminationPipeline pipeline;
    FtcDashboard dashboard;
    VisionTest.DeterminationPipeline.MarkerPosition pos;
    private double startAngle = 0;
    public static int extra = -4;
    public static double power = 0.3;

    @Override
    public void runOpMode() throws InterruptedException{
        robot = new Crab(this);

        robot.getManip().mechGrab();
        robot.getManip().setArmRotatorPower(0.3);
        robot.getTurret().setTurretPower(0.2);
        robot.getDrivetrain().lowerOdom();
        startAngle = robot.getSensors().getFirstAngle();
        dashboard = FtcDashboard.getInstance();

        pipeline = new VisionTest.DeterminationPipeline();
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

        pipeline.setSide(false);
        while (!isStarted()) {
            TelemetryPacket p = new TelemetryPacket();
            dashboard.startCameraStream(robot.getSensors().getWebcam(), 30);

            telemetry.addData("pos", pipeline.getAnalysis());
            telemetry.update();
            p.put("pos", pipeline.getAnalysis());
            dashboard.sendTelemetryPacket(p);

            pos = pipeline.getAnalysis();
        }


        waitForStart();

        robot.getManip().goToPosition(48);
        Thread.sleep(1000);
        robot.getManip().rotateClawUp();
        Thread.sleep(300);
        //robot.getTurret().setPosition(-105);


        // robot.getManip().mechGrab();

        int hub_pos;

        // TODO: Fix vision
        if (pos == VisionTest.DeterminationPipeline.MarkerPosition.LEFT)
            hub_pos = 70;
        else if (pos == VisionTest.DeterminationPipeline.MarkerPosition.CENTER)
            hub_pos = 120;
        else{
            hub_pos = 220;
            extra += 1.25;
        }

        // raise arm BEFORE we move forward
        if (pos == VisionTest.DeterminationPipeline.MarkerPosition.RIGHT){
            //robot.getTurret().setPosition(-96);
            robot.getManip().setArmRotatorPower(0.3);
            for (int i = 60; i <= 220; i += 5){
                robot.getManip().goToPosition(i);
                Thread.sleep(30);
            }
        }
        else {
            robot.getManip().goToPosition(hub_pos);
        }
        robot.getManip().rotateClawUp();

        // move towards the hub
        // TODO: Move correct distance
        robot.getDrivetrain().moveInches(20 + extra, power, false, 4);
        Thread.sleep(1700);

        // drop block
        robot.getManip().mechRelease();
        Thread.sleep(1200);

        // Go back
        robot.getDrivetrain().moveInches(-16 - extra, power + 0.15, false, 3);
        robot.getManip().rotateClawDown();

        Thread.sleep(400);
        robot.getManip().goToPosition(80);

        // Move to duck
        robot.getDrivetrain().moveInchesAngleLock(20 + extra, power + 0.1, true, robot.getSensors().getFirstAngle(), 7);
        Thread.sleep(600);

        // Put arm into excalibur mode
        robot.getManip().setArmRotatorPower(0.5);
        for (int i = 160; i <= 360; i += 10){
            robot.getManip().goToPosition(i);
            Thread.sleep(32);
        }

        Thread.sleep(1000);

        double init_heading = robot.getSensors().getFirstAngle();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.milliseconds() < 6000){
            robot.getDrivetrain().spinDuck(0.3, 0.1, -0.08 * Math.PI, robot.getSensors().getFirstAngle() - init_heading, 4, true);
        }
        robot.getDrivetrain().setAllMotors(0);

        // move away from wall to allow for spin
        timer.reset();
        while (timer.milliseconds() < 800){
            TelemetryPacket p = new TelemetryPacket();
            p.put("timer", timer.milliseconds());
            dashboard.sendTelemetryPacket(p);
            robot.getDrivetrain().spinDuck(0, 0.3, Math.PI, robot.getSensors().getFirstAngle() - init_heading, 2, false);
        }

        robot.getDrivetrain().setAllMotors(0);
        Thread.sleep(600);

        TelemetryPacket p = new TelemetryPacket();
        p.put("here", "here");
        dashboard.sendTelemetryPacket(p);

        Thread.sleep(500);


        //Turn and move
        robot.getDrivetrain().turnToPID(0, robot.getSensors(), 0.3, 4);


        Thread.sleep(400);

        telemetry.addLine("we are ddoing this");
        telemetry.update();
        robot.getDrivetrain().setMotorPowers(0,0.35,0.35,0);

        Thread.sleep(400);

        robot.getDrivetrain().setAllMotors(0);

        //lower arm
        robot.getManip().setArmRotatorPower(0.1);
        for (int i = 360; i >= 100; i -= 5){
            robot.getManip().goToPosition(i);
            Thread.sleep(30);
        }

        Thread.sleep(300);

        robot.getTurret().setPosition(-80);

        // Park in storage unit
        robot.getDrivetrain().moveInchesAngleLock(20, power + 0.15, false, robot.getSensors().getFirstAngle(), 4);
        Thread.sleep(400);
        robot.getDrivetrain().moveInchesAngleLock(20, power + 0.15, false, robot.getSensors().getFirstAngle(), 4);
        Thread.sleep(600);

        robot.getDrivetrain().turnToPID(Math.PI / 2, robot.getSensors(), 0.4, 2.25);

        Thread.sleep(400);

        robot.getDrivetrain().moveInchesAngleLock(-20, power + 0.15, false, robot.getSensors().getFirstAngle(), 4);


        // Setup for teleop
        robot.getManip().rotateClawDown();
        robot.getManip().mechRelease();
        Thread.sleep(2000);
    }
}
