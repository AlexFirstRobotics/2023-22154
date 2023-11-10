package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.armsubsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.sensors.SensorSubsystem;

@Autonomous(name = "AutonBlueSpikeDetect", group = "Auton")
public class AutonBlueSpikeDetect extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ArmSubsystem armSubsystem = new ArmSubsystem(hardwareMap);
        SensorSubsystem sensorSubsystem = new SensorSubsystem(hardwareMap);

        Trajectory trajInit = drive.trajectoryBuilder(new Pose2d(-63, 35), 1.5707963267948966)
                .splineToConstantHeading(new Vector2d(-45, 35), 0)
                .build();

        Trajectory trajFrontStart = drive.trajectoryBuilder(trajInit.end())
                .strafeRight(16)
                .forward(12)
                .build();

        Trajectory trajFrontEnd = drive.trajectoryBuilder(trajFrontStart.end())
                .back(12)
                .build();

        Trajectory trajRearStart = drive.trajectoryBuilder(trajInit.end())
                .forward(20)
                .build();

        Trajectory trajRearEnd = drive.trajectoryBuilder(trajRearStart.end())
                .back(20)
                .build();

        Trajectory trajRightStart = drive.trajectoryBuilder(trajInit.end())
                .forward(20)
                .build();

        Trajectory trajRightEnd = drive.trajectoryBuilder(trajRightStart.end())
                .back(20)
                .build();


        waitForStart();

        if (isStopRequested()) return;

        armSubsystem.CloseGrabServo();
        armSubsystem.wait(1000);
        drive.followTrajectory(trajInit);
        if (sensorSubsystem.getFrontBlue() > sensorSubsystem.getRearBlue()) {
            // Front
            if (sensorSubsystem.getFrontBlue() > sensorSubsystem.getFrontGreen()/1.5) {
                // Front

                // Move to score

                drive.followTrajectory(trajFrontStart);
                armSubsystem.wait(1000);

                // Release Pixel
                armSubsystem.OpenGrabServo();
                armSubsystem.wait(1000);

                // Move Back

                drive.followTrajectory(trajFrontEnd);
                return;
            } else {
                // Right

                // Move to score
                drive.turn(-90);
                Trajectory traj = drive.trajectoryBuilder(trajInit.end())
                        .forward(20)
                        .build();
                drive.followTrajectory(trajRightStart);
                armSubsystem.wait(1000);

                // Release Pixel
                armSubsystem.OpenGrabServo();
                armSubsystem.wait(1000);

                // Move Back

                drive.followTrajectory(trajRightEnd);
                return;
            }
        } else if (sensorSubsystem.getFrontBlue() < sensorSubsystem.getRearBlue()) {
            // Rear
            if (sensorSubsystem.getRearBlue() > sensorSubsystem.getRearGreen()/1.5) {
                // Rear

                // Move to score
                drive.turn(180);

                drive.followTrajectory(trajRearStart);
                armSubsystem.wait(1000);

                // Release Pixel
                armSubsystem.OpenGrabServo();
                armSubsystem.wait(1000);

                // Move Back

                drive.followTrajectory(trajRearEnd);
                return;
            } else {
                // Right

                // Move to score
                drive.turn(-90);

                drive.followTrajectory(trajRightStart);
                armSubsystem.wait(1000);

                // Release Pixel
                armSubsystem.OpenGrabServo();

                drive.followTrajectory(trajRightEnd);
            }
        }

    }
}
