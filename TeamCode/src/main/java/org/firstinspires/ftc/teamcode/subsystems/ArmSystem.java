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

import java.util.Arrays;
import java.util.List;

/**
 * Subsistema responsável pelo controle do braço
 * seja de forma precisa, utilizando o Controlador PID,
 * ou de forma manual pelo próprio jogador
 */
public final class ArmSystem implements Subsystem {
    public final DcMotorEx armA;
    public final DcMotorEx armB;

    private int armTarget;
    private double armPidPower;
    private double armFeedforward;
    private double robotVoltage;

    // Controlador PID pros motores
    private final PDController armController;

    public enum ArmStage {
        GROUND,
        LOW,
        MEDIUM,
        HIGH
    }

    /**
     * Construtor padrão para a configuração do hardware
     * @param hardwareMap presente em todo OpMode
     */
    public ArmSystem(HardwareMap hardwareMap) {
        // configurando os atuadores dos braços
        armA = hardwareMap.get(DcMotorEx.class, "armA"); // porta 0 - expansion
        armB = hardwareMap.get(DcMotorEx.class, "armB"); // porta 1 - expansion

        // a direção é trocada para os motores da esquerda
        armA.setDirection(DcMotorSimple.Direction.FORWARD);
        armB.setDirection(DcMotorSimple.Direction.REVERSE);

        armA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // configuramos o controlador PID
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
        armFeedforward = Math.cos(UnitConversion.encoderTicksToRadians(
                armTarget, RobotConstants.HD_HEX_40_TICKS_PER_REV)) * RobotConstants.ARM_POSITION_PID.f;
        double armCommand = armPidPower + armFeedforward;
        double armCompensedPower = Range.clip(armCommand * (12.0 / robotVoltage), -1, 1);

        armA.setTargetPosition(armTarget);
        armB.setTargetPosition(armTarget);

        // enviamos o comando pra executar o movimento
        armA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armA.setPower(armCompensedPower);
        armB.setPower(armCompensedPower);
    }

   /**
     * Movimenta o braço do robô com a força escalonada.
     * Para alterar o fator, veja o {@link RobotConstants}
     * @param joystick joystick do gamepad do jogador
     */
    public void moveArmsManually(double joystick) {
        double controlPower = joystick * RobotConstants.ARMS_POWER_SCALE;
        armA.setPower(controlPower);
        armB.setPower(controlPower);
    }

    public void setArmPosition(ArmStage position) {
        switch (position) {
            case GROUND:
                armTarget = RobotConstants.ARM_GROUND_GOAL;
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

    public void setVoltage(double voltage) {
        robotVoltage = voltage;
    }

    public double getArmFeedforwardPower() {
        return armFeedforward;
    }

    public double getArmPidPower() {
        return armPidPower;
    }
}