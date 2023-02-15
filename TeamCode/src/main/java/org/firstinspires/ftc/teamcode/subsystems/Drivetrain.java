package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

/**
 * Subsistema responsável por gerenciar e comandar o chassis
 * do robô e seus motores associados
 */
public final class Drivetrain {

    /*
     * (ramalho): segue abaixo o padrão de input pra qualquer função
     * que utilize os motores da base:
     *
     *      0 - Motor Direita-Frente
     *      1 - Motor Direita-Trás
     *      2 - Motor Esquerda-Frente
     *      3 - Motor Esquerda-Trás
     *
     */

    private final List<DcMotorEx> motors;
    private final DcMotorEx motorDireitaFrente;
    private final DcMotorEx motorDireitaTras;
    private final DcMotorEx motorEsquerdaFrente;
    private final DcMotorEx motorEsquerdaTras;

    /**
     * Construtor padrão que configura o sistema de motores
     * acessando o {@link HardwareMap}
     * @param hardwareMap
     */
    public Drivetrain(HardwareMap hardwareMap) {
        motorDireitaFrente  = hardwareMap.get(DcMotorEx.class, "DF"); // porta 0 - controlHub
        motorDireitaTras    = hardwareMap.get(DcMotorEx.class, "DT"); // porta 1 - controlHub
        motorEsquerdaFrente = hardwareMap.get(DcMotorEx.class, "EF"); // porta 2 - controlhub
        motorEsquerdaTras   = hardwareMap.get(DcMotorEx.class, "ET"); // porta 3 - controlHub

        // precisamos inverter apenas os motores da esquerda
        motorEsquerdaFrente.setDirection(DcMotor.Direction.REVERSE);
        motorEsquerdaTras.setDirection(DcMotor.Direction.REVERSE);

        motors = Arrays.asList(motorDireitaFrente, motorDireitaTras, motorEsquerdaFrente, motorEsquerdaTras);

        resetEncoderWheels();

        configEncoders(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        configZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     *
     * @return
     */
    public List<DcMotorEx> getMotors() {
        return motors;
    }

    /**
     *
     * @param ID
     * @return
     */
    public DcMotorEx getMotor(int ID) {
        return  motors.get(ID);
    }

    /**
     *
     * @param DireitaFrentePower
     * @param DireitaTrasPower
     * @param EsquerdaFrentePower
     * @param EsquerdaTrasPower
     */
    public void setMotorsPower(double DireitaFrentePower, double DireitaTrasPower, double EsquerdaFrentePower,double EsquerdaTrasPower) {
        motorDireitaFrente.setPower(DireitaFrentePower);
        motorDireitaTras.setPower(DireitaTrasPower);
        motorEsquerdaFrente.setPower(EsquerdaFrentePower);
        motorEsquerdaTras.setPower(EsquerdaTrasPower);
    }

    /**
     *
     * @param power
     */
    public void setAllPower(double power) {
        motorDireitaFrente.setPower(power);
        motorDireitaTras.setPower(power);
        motorEsquerdaFrente.setPower(power);
        motorEsquerdaTras.setPower(power);
    }

    /**
     *
     * @param runMode
     */
    public void configEncoders(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    /**
     *
     * @param ZeroPowerBehavior
     */
    public void configZeroPowerBehavior(DcMotor.ZeroPowerBehavior ZeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(ZeroPowerBehavior);
        }
    }

    /**
     * Reinicia a contagem absoluta dos encoders
     */
    public void resetEncoderWheels() {
        configEncoders(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}