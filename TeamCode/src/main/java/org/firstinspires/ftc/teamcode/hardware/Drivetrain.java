package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.Arrays;
import java.util.List;

public final class Drivetrain {
    public enum Mode {
        MINIMAL,
        FULL
    }

    private final List<DcMotorEx> motors;
    private final DcMotorEx motorDireitaFrente;
    private final DcMotorEx motorDireitaTras;
    private final DcMotorEx motorEsquerdaFrente;
    private final DcMotorEx motorEsquerdaTras;
    private final Drivetrain.Mode drivetrainMode;
    private IMU sensorIMU;
    private IMU.Parameters ParametrosDoIMU;

    public Drivetrain(HardwareMap hardwareMap, Drivetrain.Mode drivetrainMode) {
        motorDireitaFrente  = hardwareMap.get(DcMotorEx.class, "DF"); // porta 0 - controlHub
        motorDireitaTras    = hardwareMap.get(DcMotorEx.class, "DT"); // porta 1 - controlHub
        motorEsquerdaFrente = hardwareMap.get(DcMotorEx.class, "EF"); // porta 2 - controlhub
        motorEsquerdaTras   = hardwareMap.get(DcMotorEx.class, "ET"); // porta 3 - controlHub

        // precisamos inverter apenas os motores da esquerda
        motorEsquerdaFrente.setDirection(DcMotor.Direction.REVERSE);
        motorEsquerdaTras.setDirection(DcMotor.Direction.REVERSE);

        resetEncoderWheels();

        configEncoders(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        configZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motors = Arrays.asList(motorDireitaFrente, motorDireitaTras, motorEsquerdaFrente, motorEsquerdaTras);


        this.drivetrainMode = drivetrainMode;

        // OBS: o modo "FULL" do drivetrain
        if(drivetrainMode == Mode.FULL) {

            ParametrosDoIMU = new IMU.Parameters(
                    // NOTE (ramalho): aqui é de acordo com a posição que colocamos o hub no robô
                    // então é mais provável variar a direção das entradas USB nas futuras modificações do robô
                    // referências: https://ftc-docs.firstinspires.org/programming_resources/imu/imu.html

                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                    )
            );

            // configurando e carregando o sensor IMU
            sensorIMU = hardwareMap.get(IMU.class, "imu");
            sensorIMU.initialize(ParametrosDoIMU);
        }
    }

    public IMU getSensorIMU() {
        if(drivetrainMode != Mode.FULL) {
            throw new RuntimeException("ERRO: O Modo do drivetrain NÃO está configurado como 'FULL' ");
        }

        return sensorIMU;
    }

    public List<DcMotorEx> getMotors() {
        return motors;
    }

    public Mode getDrivetrainMode() {
        if(drivetrainMode != Mode.FULL) {
            throw new RuntimeException("ERRO: O Modo do drivetrain NÃO está configurado como 'FULL' ");
        }
        return drivetrainMode;
    }

    public IMU.Parameters getParametrosDoIMU() {
        return ParametrosDoIMU;
    }

    public void setMotorsPower(double DireitaFrentePower, double DireitaTrasPower, double EsquerdaFrentePower,double EsquerdaTrasPower) {
        motorDireitaFrente.setPower(DireitaFrentePower);
        motorDireitaTras.setPower(DireitaTrasPower);
        motorEsquerdaFrente.setPower(EsquerdaFrentePower);
        motorEsquerdaTras.setPower(EsquerdaTrasPower);
    }

    public void configEncoders(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void configZeroPowerBehavior(DcMotor.ZeroPowerBehavior ZeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(ZeroPowerBehavior);
        }
    }

    // Reinicia a contagem absoluta dos encoders
    public void resetEncoderWheels() {
        configEncoders(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}