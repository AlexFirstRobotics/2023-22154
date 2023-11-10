package org.firstinspires.ftc.teamcode.armsubsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

public class ArmSubsystem {

    private DcMotorEx pivotMotor, extendMotor;
    private Servo grabServo, launchServo;

    private List<Integer> lastEncoderPositions = new ArrayList<>();
    private List<Integer> lastEncoderVelocities = new ArrayList<>();

    private MotorConfigurationType pivotMotorConfig;
    private MotorConfigurationType extendMotorConfig;

    private int waitTime = 1000;

    public ArmSubsystem(HardwareMap hardwareMap) {
        pivotMotor = hardwareMap.get(DcMotorEx.class, "pivotMotor");
        extendMotor = hardwareMap.get(DcMotorEx.class, "extendMotor");

        grabServo = hardwareMap.get(Servo.class, "grabServo");
        launchServo = hardwareMap.get(Servo.class, "launchServo");

        pivotMotorConfig = pivotMotor.getMotorType().clone();
        extendMotorConfig = extendMotor.getMotorType().clone();

        pivotMotorConfig.setAchieveableMaxRPMFraction(1.0);
        extendMotorConfig.setAchieveableMaxRPMFraction(1.0);

        pivotMotor.setMotorType(pivotMotorConfig);
        extendMotor.setMotorType(extendMotorConfig);

        pivotMotor.setTargetPosition(0);
        extendMotor.setTargetPosition(0);

        pivotMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extendMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void wait(int sleeptime) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.milliseconds() < sleeptime) {

        }
    }

    public void ApplyPower(double asyncPower) {
        pivotMotor.setPower(asyncPower);
        extendMotor.setPower(asyncPower);
    }

    public void HomePosition() {
        extendMotor.setTargetPosition(ArmConstants.ARM_POSITIONS.EXTEND.HOME_POSITION);
        wait(waitTime);
        pivotMotor.setTargetPosition(ArmConstants.ARM_POSITIONS.PIVOT.HOME_POSITION);
    }

    public void ScorePosition() {
        pivotMotor.setTargetPosition(ArmConstants.ARM_POSITIONS.PIVOT.SCORE_POSITION);
        wait(waitTime);
        extendMotor.setTargetPosition(ArmConstants.ARM_POSITIONS.EXTEND.SCORE_POSITION);
    }

    public void PickupPosition() {
        pivotMotor.setTargetPosition(ArmConstants.ARM_POSITIONS.PIVOT.PICKUP_POSITION);
        wait(waitTime);
        extendMotor.setTargetPosition(ArmConstants.ARM_POSITIONS.EXTEND.PICKUP_POSITION);
    }

    public void ArmUpForClimb() {
        extendMotor.setTargetPosition(ArmConstants.ARM_POSITIONS.EXTEND.HOME_POSITION);
        pivotMotor.setTargetPosition(ArmConstants.ARM_POSITIONS.PIVOT.CLIMB_UP_POSITION);
    }

    public void ScoreExtraPosition() {
        pivotMotor.setTargetPosition(ArmConstants.ARM_POSITIONS.PIVOT.SCORE_EXTRA_POSITION);
        wait(waitTime);
        extendMotor.setTargetPosition(ArmConstants.ARM_POSITIONS.EXTEND.SCORE_POSITION);
    }

    public void Climb() {
//        pivotMotor.setPower(0.8);
        pivotMotor.setTargetPosition(ArmConstants.ARM_POSITIONS.PIVOT.CLIMB_DOWN_POSITION);
    }

    public void OpenGrabServo() {
        grabServo.setPosition(ArmConstants.ARM_POSITIONS.GRAB.OPEN);
    }

    public void CloseGrabServo() {
        grabServo.setPosition(ArmConstants.ARM_POSITIONS.GRAB.CLOSE);
    }

    public void LaunchPlane() {
        launchServo.setPosition(0.5);
    }

    public void ExtendArm(double power) { extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); extendMotor.setPower(power);}
    public void PivotArm(double power) { pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); pivotMotor.setPower(power);}

    public int GetPivotPosition() {
        return pivotMotor.getCurrentPosition();
    }

    public int GetExtendPosition() {
        return extendMotor.getCurrentPosition();
    }

    public double GetGrabPosition() {
        return grabServo.getPosition();
    }

}