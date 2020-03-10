package org.firstinspires.ftc.teamcode.drive.opmode;

import android.renderscript.ScriptIntrinsicYuvToRGB;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.MathUtil;

/**
 * TeleOp class, contains separate functions that run the robot during driver operated period of the game
 * @author Aidan Ferry
 * @since 9/8/2019
 */

@TeleOp(group = "drive")
public class Teleop extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //construct drive class
    SampleMecanumDriveREVOptimized drive;

    //Lift Arm States
    public static int[] states = new int[]{0, -170};
    public static int state=0;

    //Lift Motor State
    public static int liftState=0;

    //Button Timers
    public static long a2timer=0;
    public static long b2timer=0;
    public static long y2timer=0;

    @Override
    public void init() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
    }

    @Override
    public void loop() {
        long cur = System.currentTimeMillis();
        driveUpdate1();
        driveUpdate2(cur-a2timer>500,cur-b2timer>500, cur-y2timer>500);
        intakeUpdate();
        telemetry.update();
    }
    public double[] calcVelocities(double lx, double ly, double rx, double ry) {
        double v1 = ly + rx + lx + ry;
        double v2 = ly - rx - lx + ry;
        double v3 = ly + rx - lx + ry;
        double v4 = ly - rx + lx + ry;
        double max = Math.abs(v1);
        if(Math.abs(v2) > max)
            max = Math.abs(v2);
        if(Math.abs(v3) > max)
            max = Math.abs(v3);
        if(Math.abs(v4) > max)
            max = Math.abs(v4);
        if(max > 1) {
            v1 /= max;
            v2 /= max;
            v3 /= max;
            v4 /= max;
        }
        double[] velocities = {v1, v2, v3, v4};
        return velocities;
    }

    @Override
    public void stop() {
    }
    public void intakeUpdate(){
        float left_trigger = gamepad1.left_trigger;//in
        float right_trigger = gamepad1.right_trigger;//out2
        if(left_trigger>right_trigger){//push out
            drive.setIntakePowers(-0.75f*left_trigger);
        }else{//suck in
            drive.setIntakePowers(right_trigger);
        }
    }
    public void driveUpdate1(){
        float lx = gamepad1.left_stick_x;
        float ly = gamepad1.left_stick_y;
        float rx = gamepad1.right_stick_x;
        float ry = gamepad1.right_stick_y;
        boolean gas = gamepad1.left_bumper;
        float mult = !gas ? 1f : 0.4f;
        double[] velocities = calcVelocities(-rx*mult, ly*mult, -lx*mult, ry*mult);
        drive.setMotorPowers(velocities[0],velocities[2],velocities[3],velocities[1]);
    }
    public void driveUpdate2(boolean a2Ready, boolean b2Ready, boolean y2Ready){
        boolean trayPuller = gamepad2.a;
        boolean togglePos = gamepad2.b;
        float ry = gamepad2.right_stick_y;
        float ly = gamepad2.left_stick_y;
        if(trayPuller && a2Ready){
            drive.flipTrayPullers();
            a2timer = System.currentTimeMillis();
        }
        if(togglePos && b2Ready){
            state=1-state;
            drive.setLiftArmPosition(states[state]);
            b2timer=System.currentTimeMillis();
        }
        drive.setLiftMotorPowerRestricted(ly);
        /*if(liftLevel && y2Ready){
            liftState++;
            if(liftState==5)liftState=0;
            drive.setLiftMotorPosition(1500*liftState);
            y2timer = System.currentTimeMillis();
        }*/
        telemetry.addData("Lift Position: ",drive.getLiftMotorPosition());
        telemetry.addData("Lift Arm Position: ",drive.getLiftArmPosition());
    }
}