package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double SIDE_TICKS_PER_REV = 8192; // rev Bore value (side encoders)


    public static double FRONT_TICKS_PER_REV = 2000; // gobilda encoder val (front Encoder)

    public static double SIDE_WHEEL_RADIUS = 0.6875; // in
    public static double FRONT_WHEEL_RADIUS = 0.945; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double X_MULTIPLIER = 1.022; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1.004; // Multiplier in the Y direction

    //TODO: change lateral distance and forward offset
    public static double LATERAL_DISTANCE = 12.68; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -.5; // in; offset of the lateral wheel

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    private List<Integer> lastEncPositions, lastEncVels;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        lastEncPositions = lastTrackingEncPositions;
        lastEncVels = lastTrackingEncVels;

        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frmAndRightForwardEncoder"));
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "flmAndLeftForwardEncoder"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "blmAndStrafeEncoder"));
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        leftEncoder.setDirection(Encoder.Direction.REVERSE);

        frontEncoder.setDirection(Encoder.Direction.REVERSE);



        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double sideEncoderTicksToInches(double ticks) {
        return SIDE_WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / SIDE_TICKS_PER_REV;
    }

    public static double frontEncoderTicksToInches(double ticks) {
        return FRONT_WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / FRONT_TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int leftPos = leftEncoder.getCurrentPosition();
        int rightPos = rightEncoder.getCurrentPosition();
        int frontPos = frontEncoder.getCurrentPosition();

        lastEncPositions.clear();
        lastEncPositions.add(leftPos);
        lastEncPositions.add(rightPos);
        lastEncPositions.add(frontPos);

        return Arrays.asList(
                sideEncoderTicksToInches(leftPos) * X_MULTIPLIER,
                sideEncoderTicksToInches(rightPos) * X_MULTIPLIER,
                frontEncoderTicksToInches(frontPos) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        int leftVel = (int) leftEncoder.getCorrectedVelocity();
        int rightVel = (int) rightEncoder.getCorrectedVelocity();
        int frontVel = (int) frontEncoder.getCorrectedVelocity();

        lastEncVels.clear();
        lastEncVels.add(leftVel);
        lastEncVels.add(rightVel);
        lastEncVels.add(frontVel);

        return Arrays.asList(
                sideEncoderTicksToInches(leftVel)* X_MULTIPLIER,
                sideEncoderTicksToInches(rightVel)* X_MULTIPLIER,
                frontEncoderTicksToInches(frontVel)* Y_MULTIPLIER
        );
    }
}