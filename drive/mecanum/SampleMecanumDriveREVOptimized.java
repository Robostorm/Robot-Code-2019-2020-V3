package org.firstinspires.ftc.teamcode.drive.mecanum;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.getMotorVelocityF;

import android.support.annotation.NonNull;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.drive.localizer.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.jetbrains.annotations.NotNull;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;
import org.openftc.revextensions2.RevBulkData;

/*
 * Optimized mecanum drive implementation for REV ExHs. The time savings may significantly improve
 * trajectory following performance with moderate additional complexity.
 */
public class SampleMecanumDriveREVOptimized extends SampleMecanumDriveBase {
    private ExpansionHubEx conhub;
    private ExpansionHubEx exphub;
    private ExpansionHubMotor leftFront, leftRear, rightRear, rightFront, leftIntake, rightIntake, liftArm, liftMotor;
    private ExpansionHubServo leftGrab, rightGrab, leftArm, rightArm, leftTray, rightTray, leftIntakeDropper, rightIntakeDropper;

    private List<ExpansionHubMotor> motors;
    private List<ExpansionHubMotor> intake_motors;
    private List<ExpansionHubMotor> lift_motors;
    private List<ExpansionHubServo> servos;
    private BNO055IMU imu;

    public SampleMecanumDriveREVOptimized(HardwareMap hardwareMap) {
        super();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);
        // for simplicity, we assume that the desired IMU and drive motors are on the same hub
        // if your motors are split between hubs, **you will need to add another bulk read**
        conhub = hardwareMap.get(ExpansionHubEx.class, "Control Hub");
        exphub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hardwareMap.get(ExpansionHubMotor.class, "front_left");
        leftRear = hardwareMap.get(ExpansionHubMotor.class, "rear_left");
        rightRear = hardwareMap.get(ExpansionHubMotor.class, "rear_right");
        rightFront = hardwareMap.get(ExpansionHubMotor.class, "front_right");
        leftIntake = hardwareMap.get(ExpansionHubMotor.class, "intake_left");
        rightIntake = hardwareMap.get(ExpansionHubMotor.class, "intake_right");
        liftArm = hardwareMap.get(ExpansionHubMotor.class, "lift_arm");
        liftMotor = hardwareMap.get(ExpansionHubMotor.class, "lift_motor");

        leftGrab = hardwareMap.get(ExpansionHubServo.class,"leftGrab");
        rightGrab = hardwareMap.get(ExpansionHubServo.class,"rightGrab");
        leftArm = hardwareMap.get(ExpansionHubServo.class,"leftArm");
        rightArm = hardwareMap.get(ExpansionHubServo.class,"rightArm");
        leftTray = hardwareMap.get(ExpansionHubServo.class, "leftTray");
        rightTray = hardwareMap.get(ExpansionHubServo.class, "rightTray");
        leftIntakeDropper = hardwareMap.get(ExpansionHubServo.class, "leftIntakeDropper");
        rightIntakeDropper = hardwareMap.get(ExpansionHubServo.class, "rightIntakeDropper");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        intake_motors = Arrays.asList(leftIntake,rightIntake);
        lift_motors = Arrays.asList(liftArm,liftMotor);
        servos = Arrays.asList(leftGrab,rightGrab,leftArm,rightArm,leftTray,rightTray,leftIntakeDropper,rightIntakeDropper);

        for (ExpansionHubMotor motor : motors) {
            if (RUN_USING_ENCODER) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        for(ExpansionHubMotor motor : intake_motors){
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        for(ExpansionHubMotor motor : lift_motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        leftGrab.setPosition(0.4);//0.4 is open, 0.6 is closed
        rightGrab.setPosition(0.6);//0.6 is open, 0.4 is closed
        leftArm.setPosition(1);//0 is down, 1 is up
        rightArm.setPosition(0); //0 is up, 1 is down
        leftTray.setPosition(0);//0 is up, 1 is down
        rightTray.setPosition(1);//0 is down, 1 is up
        leftIntakeDropper.setPosition(0);//TEST
        rightIntakeDropper.setPosition(0);//TEST

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);

        //setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
    }

    public List<ExpansionHubServo> getServos(){
        return servos;
    }
    public List<ExpansionHubMotor> getMotors(){
        return motors;
    }

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }
    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (ExpansionHubMotor motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, getMotorVelocityF()
            ));
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        RevBulkData bulkData = conhub.getBulkInputData();

        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        List<Double> wheelPositions = new ArrayList<>();
        for (ExpansionHubMotor motor : motors) {
            wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(motor)));
        }
        return wheelPositions;
    }
    @Override
    public List<Double> getWheelVelocities() {
        RevBulkData bulkData = conhub.getBulkInputData();

        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        List<Double> wheelVelocities = new ArrayList<>();
        for (ExpansionHubMotor motor : motors) {
            wheelVelocities.add(encoderTicksToInches(bulkData.getMotorVelocity(motor)));
        }
        return wheelVelocities;
    }

    //LIFT ARM
    public void setLiftArmPosition(int pos){
        liftArm.setTargetPosition(pos);
        liftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftArm.setPower(1);
    }
    public void setLiftArmPower(float pow){
        liftArm.setPower(pow);
    }
    public int getLiftArmPosition(){
        return liftArm.getCurrentPosition();
    }

    //LIFT MOTOR
    public void setLiftMotorPower(float pow){
        liftMotor.setPower(pow);
    }
    public void setLiftMotorPowerRestricted(float pow){
        if((pow>0 && liftMotor.getCurrentPosition()<6000) || pow<0 && liftMotor.getCurrentPosition()>0){
            liftMotor.setPower(pow);
        }
        liftMotor.setPower(0);
    }
    public int getLiftMotorPosition(){
        return liftMotor.getCurrentPosition();
    }
    public void setLiftMotorPosition(int pos){
        liftMotor.setTargetPosition(pos);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1);
    }

    //BLOCK GRABBER ARMS
    public void leftGrabberArmDown(){
        leftArm.setPosition(0);
    }
    public void leftGrabberArmUp(){
        leftArm.setPosition(1);
    }
    public void leftGrabberArmOverTray(){
        leftArm.setPosition(0.3);
    }
    public void leftGrabberArmFloat(){
        leftArm.setPosition(0.1);
    }
    public void rightGrabberArmDown(){
        rightArm.setPosition(1);
    }
    public void rightGrabberArmUp(){
        rightArm.setPosition(0);
    }
    public void rightGrabberArmOverTray(){
        rightArm.setPosition(0.7);
    }
    public void rightGrabberArmFloat(){
        rightArm.setPosition(0.9);
    }

    //BLOCK GRABBERS
    public void leftGrabberGrab(){
        leftGrab.setPosition(0.6);
    }
    public void leftGrabberRelease(){
        leftGrab.setPosition(0.4);
    }
    public void rightGrabberGrab(){
        rightGrab.setPosition(0.4);
    }
    public void rightGrabberRelease(){
        rightGrab.setPosition(0.6);
    }

    public void intakeDroppersOut(){
        leftIntakeDropper.setPosition(1);
        rightIntakeDropper.setPosition(1);
    }
    public void intakeDroppersIn(){
        leftIntakeDropper.setPosition(0);
        rightIntakeDropper.setPosition(0);
    }

    public void setIntakePowers(float p){
        leftIntake.setPower(p);
        rightIntake.setPower(-p);
    }

    public void flipTrayPullers(){
        leftTray.setPosition(1-leftTray.getPosition());
        rightTray.setPosition(1-rightTray.getPosition());
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
}
