package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.UnitConversion;

public final class ForearmSystem implements Subsystem {

    public final DcMotorEx armC;
    public final DcMotorEx armD;

    private final PDController forearmController;

    private int forearmTarget;

    private double forearmPidPower;

    private double forearmFeedforward;

    private double robotVoltage;

    public enum ForearmStage {
        CLOSED,
        LOW,
        MEDIUM
    }

    public ForearmSystem(HardwareMap hardwareMap) {
        armC = hardwareMap.get(DcMotorEx.class, "armC"); // porta 2 - expansion
        armD = hardwareMap.get(DcMotorEx.class, "armD"); // porta 3 - expansion

        armC.setDirection(DcMotorSimple.Direction.REVERSE);
        armD.setDirection(DcMotorSimple.Direction.FORWARD);

        armC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        forearmController = new PDController(
                RobotConstants.FOREARM_POSITION_PID.p,
                RobotConstants.FOREARM_POSITION_PID.d
        );
    }

    @Override
    public void periodic() {
        int positionC = armC.getCurrentPosition();
        int positionD = armD.getCurrentPosition();

        // controle PID + feedforward do antebraço
        forearmPidPower = forearmController.calculate(positionC, forearmTarget);
        forearmFeedforward = Math.cos(UnitConversion.encoderTicksToRadians(
                forearmTarget, RobotConstants.CORE_HEX_TICKS_PER_REV
        )) * RobotConstants.FOREARM_POSITION_PID.f;

        double forearmCommand = forearmPidPower + forearmFeedforward;
        double forearmCompensedPower = Range.clip(forearmCommand * (12.0 / robotVoltage), -1, 1);

        armC.setTargetPosition(forearmTarget);
        armD.setTargetPosition(forearmTarget);

        armC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armD.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armC.setPower(forearmCompensedPower);
        armD.setPower(forearmCompensedPower);

    }

    public void moveForearmManually(double joystick) {
        double controlPower = joystick * RobotConstants.FOREARM_POWER_SCALE;
        armC.setPower(controlPower);
        armD.setPower(controlPower);
    }

    public void setForearmPosition(ForearmStage position) {
        switch (position) {
            case CLOSED:
                forearmTarget = RobotConstants.FOREARM_CLOSED;
                break;
            case LOW:
                forearmTarget = RobotConstants.FOREARM_LOW_GOAL;
                break;
            case MEDIUM:
                forearmTarget = RobotConstants.FOREARM_MEDIUM_GOAL;
                break;
        }
    }

    public double getForearmFeedforwardPower() {
        return forearmFeedforward;
    }

    public double getForearmPidPower() {
        return forearmPidPower;
    }
}
