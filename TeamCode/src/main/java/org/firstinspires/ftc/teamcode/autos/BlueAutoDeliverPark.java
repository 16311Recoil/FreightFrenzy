package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Crab;
import org.firstinspires.ftc.teamcode.VisionTestBlueWarehouse;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="BlueAutoDeliverPark", group="Auto")
public class BlueAutoDeliverPark extends LinearOpMode {
    Crab robot;
    VisionTestBlueWarehouse.DeterminationPipeline pipeline;
    FtcDashboard dashboard;
    VisionTestBlueWarehouse.DeterminationPipeline.MarkerPosition pos;
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

        pipeline = new VisionTestBlueWarehouse.DeterminationPipeline();
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

        robot.getTurret().setPosition(-96);
        robot.getManip().goToPosition(-380, -96);
        Thread.sleep(1000);
        robot.getManip().rotateClawUp();
        Thread.sleep(300);


        // robot.getManip().mechGrab();

        int hub_pos;

        // TODO: Fix vision
        if (pos == VisionTestBlueWarehouse.DeterminationPipeline.MarkerPosition.LEFT)
            hub_pos = 57;
        else if (pos == VisionTestBlueWarehouse.DeterminationPipeline.MarkerPosition.CENTER)
            hub_pos = 120;
        else{
            hub_pos = 220;
            extra += 1.25;
        }

        // raise arm BEFORE we move forward
        if (pos == VisionTestBlueWarehouse.DeterminationPipeline.MarkerPosition.RIGHT){
            robot.getTurret().setPosition(-96);
            robot.getManip().setArmRotatorPower(0.3);
            robot.getManip().goToPosition(-2500, -96);
        }
        else {
            robot.getManip().goToPosition(hub_pos, 0);
        }
        robot.getManip().rotateClawUp();

        // move towards the hub
        // TODO: Move correct distance
        robot.getDrivetrain().moveInches(21 + extra, power, false, 4);
        Thread.sleep(1700);
        robot.getDrivetrain().turnToPID(-Math.PI / 4, robot.getSensors(), 0.4, 2);
        // drop block
        robot.getManip().mechRelease();
        Thread.sleep(1200);
        robot.getDrivetrain().turnToPID(0, robot.getSensors(), 0.4, 2);

        // Go back
        robot.getDrivetrain().moveInches(-22 - extra, power + 0.15, false, 3);
        robot.getManip().rotateClawDown();

        Thread.sleep(400);
        robot.getManip().goToPosition(-800, 0);

        // Move to park
        robot.getDrivetrain().moveInchesAngleLock(-5, power + 0.1, true, robot.getSensors().getFirstAngle(), 7);
        Thread.sleep(600);


        Thread.sleep(600);

        TelemetryPacket p = new TelemetryPacket();
        p.put("here", "here");
        dashboard.sendTelemetryPacket(p);

        robot.getDrivetrain().setMotorPowers(0,0.35,0.35,0);

        Thread.sleep(900);

        robot.getDrivetrain().moveInchesAngleLock(-16, power + 0.25, true, robot.getSensors().getFirstAngle(), 5);

        Thread.sleep(600);

        // Setup for teleop
        robot.getManip().rotateClawDown();
        robot.getManip().mechRelease();
        Thread.sleep(2000);
    }
}
