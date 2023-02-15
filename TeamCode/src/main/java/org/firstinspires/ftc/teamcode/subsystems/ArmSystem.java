package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.DestemidosBot;
import org.firstinspires.ftc.teamcode.hardware.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.UnitConversion;

import java.util.Arrays;
import java.util.List;

/**
 * Subsistema responsável pela
 */
public final class ArmSystem {
    // Lista dos Atuadores e seus motores separados
    public final List<DcMotorEx> atuadores;
    public final DcMotorEx motorArmA;
    public final DcMotorEx motorArmB;

    // Controlador PID pros motores
    private final PIDController armController;

    public ArmSystem(HardwareMap hardwareMap) {
        // configurando os atuadores dos braços
        motorArmA = hardwareMap.get(DcMotorEx.class,"braçoA"); // porta 1 - expansion
        motorArmB = hardwareMap.get(DcMotorEx.class,"braçoB"); // porta 2 - expansion

        motorArmA.setDirection(DcMotorSimple.Direction.REVERSE);
        motorArmB.setDirection(DcMotorSimple.Direction.FORWARD);

        // configurando qual será o comportamento do motor, quando a força for 0
        // nesse caso, configuramos para que os motores travem na posição em que pararm
        motorArmA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArmB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // resetando todos os encoders do braço
        resetArmsEncoder();

        // inicalizando a lista de atuadores já configurados
        atuadores = Arrays.asList(motorArmA, motorArmB);

        // configuramos o controlador PID
        armController = new PIDController(
                RobotConstants.ARM_POSITION_PID.p,
                RobotConstants.ARM_POSITION_PID.i,
                RobotConstants.ARM_POSITION_PID.d
        );
    }

    public void moveArms(Gamepad driver) {
        double controlPower = driver.left_stick_y * RobotConstants.ARMS_POWER_SCALE;
        motorArmA.setPower(controlPower);
        motorArmB.setPower(controlPower);
    }

    public void moveToAngleWithPID(double targetAngle) {

        // recebemos a posição de um dos motores em ticks
        int armACurrentPosition = motorArmA.getCurrentPosition();
        //int armBCurrentPosition = motorArmB.getCurrentPosition();

        // convertemos o ângulo alvo para o equivalente em ticks
        int target_in_ticks = UnitConversion.degreesToEncoderTicks(targetAngle, RobotConstants.HD_HEX_TICKS);

        // calculamos a ação do pid e o impulso do feedforward
        // nosso setpoint e o valor medido serão em ticks
        double pidOutput = armController.calculate(armACurrentPosition, target_in_ticks);
        double ff = Math.cos(Math.toRadians(targetAngle)) * RobotConstants.ARM_POSITION_PID.f;

        double commandPower = pidOutput + ff;

        // enviamos a posição alvo e a força calculada pelo controlador
        motorArmA.setTargetPosition(target_in_ticks);
        motorArmB.setTargetPosition(target_in_ticks);

        motorArmA.setPower(commandPower);
        motorArmB.setPower(commandPower);

        // enviamos o comando pra executar o movimento
        motorArmA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArmB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // reinicia a contagem relativa dos encoders do braço
    public void resetArmsEncoder(){
        motorArmA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArmB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


}
