package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Subsistema dedicado a informar e localizar
 * o robô na arena, utilizando o sensor IMU embutido
 * no ControlHub.
 */
public class LocalizationSystem {
    final private IMU sensorIMU;
    final private IMU.Parameters imuParameters;
    private Orientation robotOrientation;

    /**
     * Construtor padrão que configura todo o sistema
     * @param hardwareMap presente em todo OpMode
     * @param sensorName nome do sensor definido no app do DriveStation
     */
    public LocalizationSystem(HardwareMap hardwareMap, String sensorName) {
        sensorIMU = hardwareMap.get(IMU.class, sensorName);

        // NOTE (ramalho): aqui é de acordo com a posição que colocamos o hub no robô
        // então é mais provável variar a direção das entradas USB nas futuras modificações do robô
        // referências: https://ftc-docs.firstinspires.org/programming_resources/imu/imu.html
        imuParameters = new IMU.Parameters(

                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT)
        );
        sensorIMU.initialize(imuParameters);
    }

    /**
     * Construtor simplificado que já assume o nome do sensor
     * que está presente no app do DriveStation
     * @param hardwareMap
     */
    public LocalizationSystem(HardwareMap hardwareMap) {
        this(hardwareMap, "imu");
    }

    /**
     * Retorna o objeto do sensor
     */
    public IMU getSensorIMU() {
        return sensorIMU;
    }

    /**
     * Retorna os parâmetros que configuraram o sensor
     */
    public IMU.Parameters getParametrosDoIMU() {
        return imuParameters;
    }

    /**
     * Atualiza todo o sistema e as principais
     * informações medidas pelo sensor
     */
    public void update() {

        // sempre damos preferência aos radianos
        robotOrientation = getRobotOrientation(AngleUnit.RADIANS);
    }

    /**
     * Reinicia a medição do ângulo do robô
     */
    public void resetAngle() {
        sensorIMU.resetYaw();
    }

    /**
     * Retorna o Ângulo em que o robô está apontado
     * neste momento, em radianos
     */
    public double getRobotHeading() {
        return getRobotOrientation(AngleUnit.RADIANS).firstAngle;
    }

    /**
     * Retorna o conjunto de informações que compôe
     * a orientação do robô em relação aos ângulos
     * @param angleUnit unidade de medida: graus ou radianos
     */
    public Orientation getRobotOrientation(AngleUnit angleUnit) {
        return sensorIMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, angleUnit);
    }
}
