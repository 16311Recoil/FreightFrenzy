package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Crab;
import org.firstinspires.ftc.teamcode.VisionTestRed;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name="RedAutoDeliverCycle", group="Auto")
public class RedAutoDeliverCycle extends LinearOpMode {
    Crab robot;
    VisionTestRed.DeterminationPipeline pipeline;
    FtcDashboard dashboard;
    VisionTestRed.DeterminationPipeline.MarkerPosition pos;
    private double startAngle = 0;
    public static int extra = -4;
    public static double power = 0.3;

    @Override
    public void runOpMode() throws InterruptedException{
        robot = new Crab(this);

        robot.getManip().mechGrab();
        robot.getManip().setArmRotatorPower(0.3);
        robot.getTurret().setTurretPower(0.35);
        robot.getDrivetrain().lowerOdom();
        startAngle = robot.getSensors().getFirstAngle();
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
            telemetry.update();
            p.put("pos", pipeline.getAnalysis());
            dashboard.sendTelemetryPacket(p);

            pos = pipeline.getAnalysis();
        }


        waitForStart();

        robot.getManip().goToPosition(38);
        Thread.sleep(1000);
        robot.getManip().rotateClawUp();
        Thread.sleep(300);
        //robot.getTurret().setPosition(-80);


        // robot.getManip().mechGrab();

        int hub_pos;

        // TODO: Fix vision
        if (pos == VisionTestRed.DeterminationPipeline.MarkerPosition.LEFT)
            hub_pos = 57;
        else if (pos == VisionTestRed.DeterminationPipeline.MarkerPosition.CENTER)
            hub_pos = 120;
        else{
            hub_pos = 220;
            extra += 1.25;
        }

        // raise arm BEFORE we move forward
        if (pos == VisionTestRed.DeterminationPipeline.MarkerPosition.RIGHT){
            robot.getTurret().setPosition(-69);
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
        robot.getDrivetrain().moveInches(21 + extra, power + 0.2, false, 4);
        Thread.sleep(1700);

        // drop block
        robot.getManip().mechRelease();
        Thread.sleep(1200);

        // Go back
        robot.getDrivetrain().moveInches(-22 - extra, power + 0.2, false, 3);

        Thread.sleep(400);
        robot.getManip().goToPosition(80);

        // Move to park
        robot.getDrivetrain().moveInches(5, power + 0.15, true, 7);
        Thread.sleep(600);


        // Turn to Grab
        robot.getDrivetrain().turnToPID(-Math.PI / 2, robot.getSensors(), 0.4, 2);

        Thread.sleep(600);

        TelemetryPacket p = new TelemetryPacket();
        p.put("here", "here");
        dashboard.sendTelemetryPacket(p);

        robot.getTurret().setPosition(-98);
        robot.getManip().goToPosition(0);
        robot.getDrivetrain().setMotorPowers(0.35,0,0,0.35);

        Thread.sleep(800);


        //Move and Grab
        robot.getDrivetrain().moveInchesAngleLock2(27, power + 0.25, 0.17,false, robot.getSensors(), 5);
        Thread.sleep(200);
        robot.getManip().mechGrab();
        Thread.sleep(600);

        //Move and Deliver
        robot.getManip().goToPosition(120);
        robot.getTurret().setPosition(180);
        Thread.sleep(200);
        robot.getDrivetrain().moveInchesAngleLock2(-24, power + 0.25, 0.17,false, robot.getSensors(), 5);
        Thread.sleep(400);
        robot.getDrivetrain().moveInchesAngleLock2(-18, power + 0.25, 0,true, robot.getSensors(), 5);
        Thread.sleep(200);
        robot.getManip().mechRelease();
        Thread.sleep(700);

        //Move back
        robot.getDrivetrain().moveInches(23, power + 0.2, true, 3);
        Thread.sleep(500);
        robot.getTurret().setPosition(-98);
        robot.getManip().goToPosition(0);


        robot.getDrivetrain().moveInchesAngleLock2(26, power + 0.25, 0.1,false, robot.getSensors(), 4);
        Thread.sleep(200);

        // Setup for teleop
        robot.getManip().rotateClawDown();
        robot.getManip().mechRelease();
        Thread.sleep(2000);
    }
}
