package org.firstinspires.ftc.teamcode.drive.opmode;

import android.renderscript.ScriptIntrinsicYuvToRGB;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.openftc.revextensions2.ExpansionHubServo;

import java.util.List;

/**
 * TeleOp class, contains separate functions that run the robot during driver operated period of the game
 * @author Aidan Ferry
 * @since 9/8/2019
 */

@TeleOp(group = "drive")
public class ServoTest extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //construct drive class
    SampleMecanumDriveREVOptimized drive;

    public List<ExpansionHubServo> servos;
    public static int index =0;
    public static long atimer=0;
    @Override
    public void init() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        servos = drive.getServos();
    }

    @Override
    public void loop() {
        float ly = -gamepad1.left_stick_y;
        boolean a = gamepad1.left_bumper;

        if(a && System.currentTimeMillis()-atimer > 1000){
            index++;
            if(index>servos.size()-1)index=0;
            atimer=System.currentTimeMillis();
        }
        if(System.currentTimeMillis()%50==0) {
            double newval = servos.get(index).getPosition()+(ly/10);
            servos.get(index).setPosition((newval<=1 && newval>=0) ? newval : servos.get(index).getPosition());
            telemetry.addData("Current Servo: ",index);
            telemetry.addData("Current Position: ",servos.get(index).getPosition());
            telemetry.update();
        }
    }

    @Override
    public void stop() {
    }
}
