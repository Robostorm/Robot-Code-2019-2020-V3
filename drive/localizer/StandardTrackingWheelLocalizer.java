package org.firstinspires.ftc.teamcode.drive.localizer;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;

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
 * Note: this could be optimized significantly with REV bulk reads
 */

/*
Go to StandardTrackingWheelLocalizer and fill in the values and the locations of your odometry wheels relative to THE CENTER OF THE WHEELS. Positive X is the front of the robot, Positive Y is the left of the robot.
Make sure these values are accurate, test with Localization Test. When you move the robot forwards 2 inches, is the reading X:2. When you turn the robot 90 degrees left, is the reading Heading: 1.5708 (90 degrees as radians).
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 1450;
    public static double WHEEL_RADIUS = 0.75; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 14.88; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 3.907; // in; offset of the lateral wheel

    private List<DcMotor> motors;
    private DcMotor leftEncoder, rightEncoder, frontEncoder;
    private ExpansionHubEx conhub;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(-1.9889272, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(-1.9889272, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET-1.9889272, 0, Math.toRadians(90)) // front
        ));

        conhub = hardwareMap.get(ExpansionHubEx.class, "Control Hub");

        leftEncoder = hardwareMap.dcMotor.get("front_right");
        rightEncoder = hardwareMap.dcMotor.get("front_left");
        frontEncoder = hardwareMap.dcMotor.get("rear_right");

        motors = Arrays.asList(leftEncoder,rightEncoder,frontEncoder);
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        RevBulkData bulkData = conhub.getBulkInputData();

        if(bulkData ==null) return Arrays.asList(0.0,0.0,0.0,0.0);

        List<Double> wheelPositions = new ArrayList<>();
        for(DcMotor motor : motors){
            wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(motor)));
        }

        return wheelPositions;
    }
}
