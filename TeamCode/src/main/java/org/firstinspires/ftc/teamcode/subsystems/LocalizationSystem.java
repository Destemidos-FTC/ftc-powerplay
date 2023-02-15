package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class LocalizationSystem {
    final private IMU sensorIMU;
    final private IMU.Parameters imuParameters;
    private Orientation robotOrientation;

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
     *
     * @return
     */
    public IMU getSensorIMU() {
        return sensorIMU;
    }

    /**
     *
     * @return
     */
    public IMU.Parameters getParametrosDoIMU() {
        return imuParameters;
    }

    /**
     *
     */
    public void update() {
        robotOrientation = getRobotOrientation();
    }

    /**
     *
     */
    public void resetAngle() {
        sensorIMU.resetYaw();
    }

    /**
     *
     */
    public Orientation getRobotOrientation() {
        return sensorIMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
    }



}
