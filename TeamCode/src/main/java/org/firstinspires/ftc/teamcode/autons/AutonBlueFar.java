package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.armsubsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "AutonBlueFar", group = "Auton")
public class AutonBlueFar extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ArmSubsystem arm = new ArmSubsystem(hardwareMap);

        waitForStart();

        arm.CloseGrabServo();
        arm.wait(1000);
        Trajectory traj = drive.trajectoryBuilder(new Pose2d(-38, 63), Math.toRadians(8))
                .splineTo(new Vector2d(90, 60), 0)
                .build();
        drive.followTrajectory(traj);
        arm.wait(1000);
        arm.OpenGrabServo();

    }
}
