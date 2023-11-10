package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.armsubsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "TeleOpMain", group = "TeleOp")
public class TeleOpMain extends OpMode {

    SampleMecanumDrive mecanumDrive;
    ArmSubsystem armSubsystem;
    Thread driveThread;
    int startThread = 0;

    @Override
    public void init() {
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        armSubsystem = new ArmSubsystem(hardwareMap);
        armSubsystem.ApplyPower(1);
        driveThread = new DriveThread();
    }

    private class DriveThread extends java.lang.Thread {
        public DriveThread(){
            this.setName("DriveThread");
        }

        @Override
        public void run(){
            while (!isInterrupted()) {
                mecanumDrive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x*.75,
                                -gamepad1.right_stick_x
                        )
                );
            }
        }
    }

    @Override
    public void loop() {
        if (startThread == 0) {
            driveThread.start();
            startThread = 1;
        }


        if (gamepad2.a) {
            armSubsystem.PickupPosition();
        } else if (gamepad2.b) {
            armSubsystem.HomePosition();
        } else if (gamepad2.y) {
            armSubsystem.ScorePosition();
        }

        if (gamepad2.x) {
            armSubsystem.ScoreExtraPosition();
        }

        if (gamepad2.right_bumper) {
            armSubsystem.OpenGrabServo();
        } else if (gamepad2.left_bumper) {
            armSubsystem.CloseGrabServo();
        }

        if (gamepad2.dpad_up) {
            armSubsystem.ArmUpForClimb();
        } else if (gamepad2.dpad_down) {
            armSubsystem.Climb();
        }

        if (gamepad1.a) {
            armSubsystem.LaunchPlane();
        }

    }

}
