package org.firstinspires.ftc.teamcode.drive.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@Autonomous (name="Default", group="Blue")
public class Auto extends LinearOpMode {

    public static final double robotWidth=17.52;
    public static final double robotLength=20.73;
    public static final double backLength = 8.366;
    public static final double frontLength = robotLength-backLength;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-36, 72-backLength, 270));
        waitForStart();

        if (isStopRequested()) return;
        while(!isStopRequested()){
            //Drive to 1st block from front
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                        .splineTo(new Pose2d(-28, 24+(robotWidth/2), 0)).build()
            );
            //Grab block and lift arm to float level
            grabBlock(drive);
            //Drive to center of blue bridge
            driveToBlueBridgeCenter(drive);
            //Drive to side of foundation as far forward as possible
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                        .splineTo(new Pose2d(72-frontLength, 24+robotWidth/2, 0)).build()
            );
            //Drop block on foundation
            dropBlock(drive);
            //Reverse to center of blue bridge
            reverseToBlueBridgeCenter(drive);
            //Reverse to 2nd block from front
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(-36, 24+(robotWidth/2), 0)).reverse().build()
            );
            //Grab block
            grabBlock(drive);
            //Drive to center of blue bridge
            driveToBlueBridgeCenter(drive);
            //Drive to side of foundation, one blocklength closer to center
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(72-frontLength-8, 24+robotWidth/2, 0)).build()
            );
            //Drop block onto foundation
            dropBlock(drive);
            //Reverse to center of blue bridge
            reverseToBlueBridgeCenter(drive);
            //Reverse to 3rd block from front
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(-44, 24+(robotWidth/2), 0)).reverse().build()
            );
            //Grab block
            grabBlock(drive);
            //Drive to center of blue bridge
            driveToBlueBridgeCenter(drive);
            //Drive to side of foundation, two blocklengths closer to center
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(72-frontLength-16, 24+robotWidth/2, 0)).build()
            );
            //Drop block onto foundation
            dropBlock(drive);
            //Reverse to center of blue bridge
            reverseToBlueBridgeCenter(drive);
            //Reverse to 4th block from front
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(-52, 24+(robotWidth/2), 0)).reverse().build()
            );
            //Grab Block
            grabBlock(drive);
            //Drive to center of blue bridge
            driveToBlueBridgeCenter(drive);
            //Drive to side of foundation, three blocklengths closer to center
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .splineTo(new Pose2d(72-frontLength-24, 24+robotWidth/2, 0)).build()
            );
            //Drop block
            dropBlock(drive);
            //Reverse to blue bridge center
            reverseToBlueBridgeCenter(drive);
        }
    }
    public void grabBlock(SampleMecanumDriveREVOptimized drive){
        drive.rightGrabberArmDown();
        sleep(500);
        drive.rightGrabberGrab();
        sleep(250);
        drive.rightGrabberArmFloat();
    }
    public void dropBlock(SampleMecanumDriveREVOptimized drive){
        drive.rightGrabberArmOverTray();
        sleep(250);
        drive.rightGrabberRelease();
        sleep(250);
        drive.rightGrabberArmUp();
    }
    public void driveToBlueBridgeCenter(SampleMecanumDriveREVOptimized drive){
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(0, 48, 0)).build()
        );
    }
    public void reverseToBlueBridgeCenter(SampleMecanumDriveREVOptimized drive){
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(0, 48, 0)).reverse().build()
        );
    }
}
