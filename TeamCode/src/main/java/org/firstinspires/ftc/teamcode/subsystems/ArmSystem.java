package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

/**
 * Subsistema responsável pelo controle do braço
 * seja de forma precisa, utilizando o Controlador PID,
 * ou de forma manual pelo próprio jogador
 */
public final class ArmSystem implements Subsystem {

    // Motores
    public final DcMotorEx armA;
    public final DcMotorEx forearmMotor;

    //
    private double robotVoltage;

    //
    private int armTarget;
    private int forearmTarget;

    private double armPID;
    private double forearmPID;

    private double armFeedforward;
    private double forearmFeedforward;

    // Controlador PID pros motores
    private final PDController armController;

    //
    public enum ArmStage {
        CLOSED,
        LOW,
        MEDIUM,
        HIGH
    }

    private final PDController forearmController;

    public enum ForearmStage {
        CLOSED,
        COLLECT,
        LOW,
        MEDIUM,
        HIGH
    }

    /**
     * Construtor padrão para a configuração do hardware
     * @param hardwareMap presente em todo OpMode
     */
    public ArmSystem(HardwareMap hardwareMap) {
        armA = hardwareMap.get(DcMotorEx.class, "arm"); // porta 0 - expansion
        forearmMotor = hardwareMap.get(DcMotorEx.class, "forearm"); // porta 1 - expansion

        armA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forearmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        forearmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armA.setDirection(DcMotorSimple.Direction.FORWARD);
        forearmMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        armA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        forearmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armController = new PDController(
                RobotConstants.ARM_POSITION_PID.p,
                RobotConstants.ARM_POSITION_PID.d
        );

        forearmController = new PDController(
                RobotConstants.FOREARM_POSITION_PID.p,
                RobotConstants.FOREARM_POSITION_PID.d
        );
    }

    @Override
    public void periodic() {

        // posição dos motores
        int armPosition = armA.getCurrentPosition();
        int forearmPosition = forearmMotor.getCurrentPosition();

        // controle PID + feedforward do braço
        armPID = armController.calculate(armPosition, armTarget);
        forearmPID = forearmController.calculate(forearmPosition, forearmTarget);

        armFeedforward = RobotConstants.ARM_POSITION_PID.f;
        forearmFeedforward = RobotConstants.FOREARM_POSITION_PID.f;

        double armCommand = armPID + armFeedforward;
        double forearmCommand = forearmPID + forearmFeedforward;

        double armCompensedPower = Range.clip(armCommand * (12.0 / robotVoltage), -1, 1);
        double forearmCompensedPower = Range.clip(forearmCommand * (12 / robotVoltage), -1, 1);

        if(armController.atSetPoint()) {
            armCompensedPower = 0;
        }

        if(forearmController.atSetPoint()) {
            forearmCompensedPower = 0;
        }

        armA.setTargetPosition(armTarget);
        armA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armA.setPower(armCompensedPower);

        forearmMotor.setTargetPosition(forearmTarget);
        forearmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        forearmMotor.setPower(forearmCompensedPower);
    }

   /**
     * Movimenta o braço do robô com a força escalonada.
     * Para alterar o fator, veja o {@link RobotConstants}
     * @param joystick joystick do gamepad do jogador
     */
    public void moveArmsManually(double joystick) {
        double controlPower = joystick * RobotConstants.ARMS_POWER_SCALE;
        armA.setPower(controlPower);
    }

    public void moveForearmManually(float left_stick_y) {
        double controlPower = left_stick_y * RobotConstants.FOREARM_POWER_SCALE;
        forearmMotor.setPower(controlPower);
    }

    public void setArmPosition(ArmStage position) {
        switch (position) {
            case CLOSED:
                armTarget = RobotConstants.ARM_CLOSED_GOAL;
                break;
            case LOW:
                armTarget = RobotConstants.ARM_LOW_GOAL;
                break;
            case MEDIUM:
                armTarget = RobotConstants.ARM_MEDIUM_GOAL;
                break;
            case HIGH:
                armTarget = RobotConstants.ARM_HIGH_GOAL;
                break;
        }
    }

    public void setForearmPosition(ForearmStage position) {
        switch (position) {
            case CLOSED:
                forearmTarget = RobotConstants.FOREARM_CLOSED_GOAL;
                break;
            case COLLECT:
                forearmTarget = RobotConstants.FOREARM_COLLECT_GOAL;
                break;
            case LOW:
                forearmTarget = RobotConstants.FOREARM_LOW_GOAL;
                break;
            case MEDIUM:
                forearmTarget = RobotConstants.FOREARM_MEDIUM_GOAL;
                break;
            case HIGH:
                forearmTarget = RobotConstants.FOREARM_HIGH_GOAL;
        }
    }

    //
    public void setVoltage(double voltage) {
        robotVoltage = voltage;
    }

    //
    public double getArmFeedforwardPower() {
        return armFeedforward;
    }

    //
    public double getArmPID() {
        return armPID;
    }

    //
    public double getForearmFeedforwardPower() {
        return armFeedforward;
    }

    //
    public double getForearmPID() {
        return armPID;
    }
}