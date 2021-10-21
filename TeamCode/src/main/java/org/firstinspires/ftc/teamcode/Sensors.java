package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

public class Sensors {

    public BNO055IMU gyro;
    private Orientation angles;
    private List<LynxModule> allHubs;
    private OpMode iterative_OpMode;
    private LinearOpMode linear_OpMode;
    // Used for logging cycle reads.
    private ElapsedTime readTimer;
    View relativeLayout;
    private FtcDashboard dashboard;
    public List<Encoder> encoders;

    private boolean autoBulkRead = true;

    public Sensors(LinearOpMode opMode){
        this.linear_OpMode = opMode;
        // Gets all REV Hubs
        LynxModuleUtil.ensureMinimumFirmwareVersion(linear_OpMode.hardwareMap);
        allHubs = linear_OpMode.hardwareMap.getAll(LynxModule.class);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro = linear_OpMode.hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(parameters);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public Sensors(OpMode opMode){
        this.iterative_OpMode = opMode;

        this.iterative_OpMode = opMode;
        // Gets all REV Hubs
        LynxModuleUtil.ensureMinimumFirmwareVersion(opMode.hardwareMap);
        allHubs = opMode.hardwareMap.getAll(LynxModule.class);


        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Hardware Maps All of the Encoders
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(parameters);
    }


    public List<Encoder> getEncoders() {
        return encoders;
    }

    public void setEncoders(List<Encoder> encoders) {
        this.encoders = encoders;
    }


    private void updateGyro()
    {
        angles = gyro.getAngularOrientation();
    }


    // Bulk Reads Sensor Data to update all values for use.
    public void updateSensorVals(){
        updateGyro();
    }

    // Determine which is yaw, pitch, and roll.
    public double getFirstAngle()
    {
        updateGyro();
        return angles.firstAngle;
    }
    public double getSecondAngle()
    {
        updateGyro();
        return angles.secondAngle;
    }
    public double getThirdAngle()
    {
        updateGyro();
        return angles.thirdAngle;
    }


    public FtcDashboard getDashboard() {
        return dashboard;
    }

    public void setDashboard(FtcDashboard dashboard) {
        this.dashboard = dashboard;
    }
}
