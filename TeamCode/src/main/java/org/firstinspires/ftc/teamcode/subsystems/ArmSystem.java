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

    //
    public final DcMotorEx armA;

    //
    private int armTarget;
    private double armPidPower;
    private double armFeedforward;
    private double robotVoltage;

    // Controlador PID pros motores
    private final PDController armController;

    //
    public enum ArmStage {
        CLOSED,
        LOW,
        MEDIUM,
        HIGH
    }

    /**
     * Construtor padrão para a configuração do hardware
     * @param hardwareMap presente em todo OpMode
     */
    public ArmSystem(HardwareMap hardwareMap) {
        armA = hardwareMap.get(DcMotorEx.class, "armA"); // porta 0 - expansion

        armA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armA.setDirection(DcMotorSimple.Direction.FORWARD);

        armA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armController = new PDController(
                RobotConstants.ARM_POSITION_PID.p,
                RobotConstants.ARM_POSITION_PID.d
        );
    }

    @Override
    public void periodic() {
        armController.setPID(
                RobotConstants.ARM_POSITION_PID.p,
                0,
                RobotConstants.ARM_POSITION_PID.d
        );

        // posição dos motores
        int positionA = armA.getCurrentPosition();

        // controle PID + feedforward do braço
        armPidPower = armController.calculate(positionA, armTarget);
        armFeedforward = RobotConstants.ARM_POSITION_PID.f;

        double armCommand = armPidPower + armFeedforward;
        double armCompensedPower = Range.clip(armCommand * (12.0 / robotVoltage), -1, 1);

        armA.setTargetPosition(armTarget);
        armA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armA.setPower(armCompensedPower);
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

    //
    public void setVoltage(double voltage) {
        robotVoltage = voltage;
    }

    //
    public double getArmFeedforwardPower() {
        return armFeedforward;
    }

    //
    public double getArmPidPower() {
        return armPidPower;
    }
}