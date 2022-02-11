package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Crab;
import org.firstinspires.ftc.teamcode.VisionTestBlueWarehouse;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(name="BlueGrind", group="Auto")
public class BlueAutoGrind extends LinearOpMode {

    Crab robot;
    FtcDashboard dashboard;
    VisionTestBlueWarehouse.DeterminationPipeline pipeline;
    VisionTestBlueWarehouse.DeterminationPipeline.MarkerPosition pos;

    public void runOpMode() throws InterruptedException{

        dashboard = FtcDashboard.getInstance();
        robot = new Crab(this);
        robot.getDrivetrain().lowerOdom();
        robot.getManip().magGrab();


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
            p.put("pos", pipeline.getAnalysis());
            dashboard.sendTelemetryPacket(p);

            pos = pipeline.getAnalysis();
        }

        waitForStart();

        if (pos == VisionTestBlueWarehouse.DeterminationPipeline.MarkerPosition.LEFT){
            /*robot.getManip().goToPosition(35);
            robot.getTurret().setPosition(170);
            robot.getDrivetrain().moveInches(-15.5, 0.5, false);
            Thread.sleep(1000);
            robot.getDrivetrain().moveInches(-17, 0.5, true);
            robot.getManip().magRelease();
            Thread.sleep(2000);
            robot.getDrivetrain().moveInches(4, 0.5, true);
            Thread.sleep(1000);
            robot.getTurret().setPosition(-60);
            robot.getDrivetrain().moveInches(-30, 0.5, false);
            robot.getManip().magGrab();
            Thread.sleep(1000);
            robot.getTurret().setPosition(265);
            Thread.sleep(4000);*/
        }

        else if (pos == VisionTestBlueWarehouse.DeterminationPipeline.MarkerPosition.CENTER){
            /*robot.getManip().goToPosition(110);
            robot.getTurret().setPosition(0);
            robot.getDrivetrain().moveInches(-9, 0.5, false);
            Thread.sleep(1000);
            robot.getDrivetrain().moveInches(-16, 0.5, true);
            robot.getManip().magRelease();
            Thread.sleep(2000);
            robot.getDrivetrain().moveInches(5, 0.5, true);
            Thread.sleep(1000);
            robot.getTurret().setPosition(-40);
            robot.getDrivetrain().moveInches(-37, 0.5, false);
            robot.getManip().magGrab();
            Thread.sleep(1000);
            robot.getTurret().setPosition(0);
            Thread.sleep(4000);*/
        }

        else{
            /*robot.getManip().goToPosition(200);
            robot.getTurret().setPosition(0);
            robot.getDrivetrain().moveInches(-9.5, 0.5, false); //move forward
            Thread.sleep(1000);
            robot.getDrivetrain().moveInches(-15, 0.5, true); //move right
            robot.getManip().magRelease();
            Thread.sleep(2000);
            robot.getDrivetrain().moveInches(8, 0.5, true); //move back
            Thread.sleep(1000);
            robot.getTurret().setPosition(-60);
            robot.getDrivetrain().moveInches(-53, 0.5, false); // move to depo
            robot.getManip().magGrab();
            Thread.sleep(1000);
            robot.getTurret().setPosition(0);
            Thread.sleep(1000);
            robot.getManip().goToPosition(90);
            Thread.sleep(3000);*/
        }

        //Red Bottom Partial
        /*robot.getManip().goToPosition(35);
        robot.getTurret().setPosition(0);
        robot.getDrivetrain().moveInches(-9.5, 0.5, true, opModeIsActive());
        Thread.sleep(1000);
        robot.getDrivetrain().moveInches(7, 0.5, false, opModeIsActive());
        robot.getManip().magRelease();
        Thread.sleep(2000);
        robot.getDrivetrain().moveInches(4, 0.5, true, opModeIsActive());
        Thread.sleep(1000);
        robot.getTurret().setPosition(-60);
        robot.getDrivetrain().moveInches(30, 0.5, false, opModeIsActive());
        robot.getManip().magGrab();
        Thread.sleep(5000);*/

        //Red Mid Partial
        /*robot.getManip().goToPosition(90);
        robot.getTurret().setPosition(0);
        robot.getDrivetrain().moveInches(-9.5, 0.5, true, opModeIsActive());
        Thread.sleep(1000);
        robot.getDrivetrain().moveInches(3, 0.5, false, opModeIsActive());
        robot.getManip().magRelease();
        Thread.sleep(2000);
        robot.getDrivetrain().moveInches(4, 0.5, true, opModeIsActive());
        Thread.sleep(1000);
        robot.getTurret().setPosition(-60);
        robot.getDrivetrain().moveInches(37, 0.5, false, opModeIsActive());
        robot.getManip().magGrab();
        Thread.sleep(5000);*/


        /*//Red High Partial Park
        robot.getManip().goToPosition(200);
        robot.getTurret().setPosition(0);
        robot.getDrivetrain().moveInches(-9.5, 0.5, true, opModeIsActive()); //move forward
        Thread.sleep(1000);
        robot.getDrivetrain().moveInches(3, 0.5, false, opModeIsActive()); //move right
        robot.getManip().magRelease();
        Thread.sleep(2000);
        robot.getDrivetrain().moveInches(7, 0.5, true, opModeIsActive()); //move back
        Thread.sleep(1000);
        robot.getTurret().setPosition(-60);
        robot.getDrivetrain().moveInches(37, 0.5, false, opModeIsActive()); // move to depo
        robot.getManip().magGrab();
        Thread.sleep(1000);
        robot.getTurret().setPosition(0);
        Thread.sleep(4000);*/

        /*//Red High Part Full Park
        robot.getManip().goToPosition(200);
        robot.getTurret().setPosition(0);
        robot.getDrivetrain().moveInches(-9.5, 0.5, true, opModeIsActive()); //move forward
        Thread.sleep(1000);
        robot.getDrivetrain().moveInches(3, 0.5, false, opModeIsActive()); //move right
        robot.getManip().magRelease();
        Thread.sleep(2000);
        robot.getDrivetrain().moveInches(8, 0.5, true, opModeIsActive()); //move back
        Thread.sleep(1000);
        robot.getTurret().setPosition(-60);
        robot.getDrivetrain().moveInches(52, 0.5, false, opModeIsActive()); // move to depo
        robot.getManip().magGrab();
        Thread.sleep(1000);
        robot.getTurret().setPosition(0);
        Thread.sleep(4000);*/




    }


}
