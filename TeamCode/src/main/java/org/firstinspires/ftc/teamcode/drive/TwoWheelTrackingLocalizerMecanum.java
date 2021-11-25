package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.DrivetrainRR;
import org.firstinspires.ftc.teamcode.DrivetrainRRMecanum;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->
 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
public class TwoWheelTrackingLocalizerMecanum extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.625; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = -4; // X is the up and down direction
    public static double PARALLEL_Y = -.5; // Y is the strafe direction

    public static double PERPENDICULAR_X = 0.5;
    public static double PERPENDICULAR_Y = -4;

    /*ORIGINAL VALUES

    public static double PARALLEL_X = 3.673; // X is the up and down direction
    public static double PARALLEL_Y = -2.921; // Y is the strafe direction


    public static double PERPENDICULAR_X = 2.921;
    public static double PERPENDICULAR_Y = 3.673;
    */

    /*TEST VALUES 2
    public static double PARALLEL_X = 0.53174429945228; // X is the up and down direction
    public static double PARALLEL_Y = -4.6626621151441; // Y is the strafe direction

    public static double PERPENDICULAR_X = 4.6626621151441;
    public static double PERPENDICULAR_Y = 0.53174429945228;
    */

    //TODO: Double check whether we need to care about the flip in directions for dual drivetrains
    public static double X_MULTIPLIER = 1.1314687545; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1.1142742882; // Multiplier in the Y direction

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private Encoder parallelEncoder, perpendicularEncoder;

    private DrivetrainRRMecanum drive;

    public TwoWheelTrackingLocalizerMecanum(HardwareMap hardwareMap, DrivetrainRRMecanum drive) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, Math.toRadians(-45)),//Back
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(45))//Right
        ));

        this.drive = drive;


        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "f"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "l"));
        X_MULTIPLIER = 1.1265519381; // Multiplier in the X direction
        Y_MULTIPLIER = 1.1142742882; // Multiplier in the Y direction




        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);
        parallelEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition() * Y_MULTIPLIER)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}