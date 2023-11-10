package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.armsubsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.sensors.SensorSubsystem;

@TeleOp(name = "LogAllPositions", group = "logging")
public class LogAllPositions extends OpMode {
    SampleMecanumDrive mecanumDrive;
    ArmSubsystem armSubsystem;
    SensorSubsystem sensorSubsystem;

    @Override
    public void init() {
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        armSubsystem = new ArmSubsystem(hardwareMap);
        sensorSubsystem = new SensorSubsystem(hardwareMap);
    }

    @Override
    public void loop() {

        armSubsystem.ExtendArm(-gamepad2.left_stick_y);
        armSubsystem.PivotArm(-gamepad2.right_stick_y);

        telemetry.addData("PivotPosition", armSubsystem.GetPivotPosition());
        telemetry.addData("ExtendPosition", armSubsystem.GetExtendPosition());
        telemetry.addData("GrabPosition", armSubsystem.GetGrabPosition());

        telemetry.addData("FRed", sensorSubsystem.getFront()[0]);
        telemetry.addData("FGreen", sensorSubsystem.getFront()[1]);
        telemetry.addData("FBlue", sensorSubsystem.getFront()[2]);

        telemetry.addData("RRed", sensorSubsystem.getRear()[0]);
        telemetry.addData("RGreen", sensorSubsystem.getRear()[1]);
        telemetry.addData("RBlue", sensorSubsystem.getRear()[2]);

        telemetry.update();
    }


}
