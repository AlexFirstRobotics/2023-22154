package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.armsubsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "AutonBlueNearPark", group = "Auton")
public class AutonBlueNearPark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ArmSubsystem arm = new ArmSubsystem(hardwareMap);

        waitForStart();

        arm.CloseGrabServo();
        arm.wait(1000);
        Trajectory traj = drive.trajectoryBuilder(new Pose2d(0, 0))
                .splineTo(new Vector2d(70, 0), 0)
                .build();
        drive.followTrajectory(traj);
        arm.wait(1000);
        arm.OpenGrabServo();

    }
}
