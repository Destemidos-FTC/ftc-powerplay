package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

public final class ForearmSystem implements Subsystem {

    public final DcMotorEx forearmMotor;

    private double robotVoltage;

    private int forearmTarget;

    private double forearmPID;

    private double forearmFeedforward;

    private PDController forearmController;

    public enum ForearmStage {
        CLOSED,
        LOW,
        MEDIUM
    }

    public ForearmSystem(HardwareMap hardwareMap) {
        forearmMotor = hardwareMap.get(DcMotorEx.class, "armB"); // porta 1 - expansion

        forearmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        forearmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        forearmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        forearmController = new PDController(
                RobotConstants.FOREARM_POSITION_PID.p,
                RobotConstants.FOREARM_POSITION_PID.d
        );
    }

    @Override
    public void periodic() {

        forearmController.setPID(
                RobotConstants.FOREARM_POSITION_PID.p,

                0.0,
                RobotConstants.FOREARM_POSITION_PID.d
        );

        int currentPosition = forearmMotor.getCurrentPosition();

        forearmPID = forearmController.calculate(currentPosition, forearmTarget);
        forearmFeedforward = RobotConstants.FOREARM_POSITION_PID.f;

        double power = forearmPID + forearmFeedforward;

        double forearmCompensedPower = power * (12 / robotVoltage);

        forearmMotor.setPower(forearmCompensedPower);
    }

    public void moveForearmManually(double joystick) {
        double power = joystick * RobotConstants.FOREARM_POWER_SCALE;
        forearmMotor.setPower(power);
    }

    public void setForearmPosition(ForearmStage position) {
        switch (position) {
            case CLOSED:
                forearmTarget = RobotConstants.FOREARM_CLOSED_GOAL;
                break;
            case LOW:
                forearmTarget = RobotConstants.FOREARM_LOW_GOAL;
                break;
            case MEDIUM:
                forearmTarget = RobotConstants.FOREARM_MEDIUM_GOAL;
                break;
        }
    }

    public void setVoltage(double voltage) {
        robotVoltage = voltage;
    }
}
