package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.DestemidosHardware;
import org.firstinspires.ftc.teamcode.hardware.RobotConstants;

public final class ArmSystem {

    public static void movimentarBraço(Gamepad driver, DestemidosHardware robot)
    {
        // Gira a base do robô
        double motorCenterOutput = Range.clip(-driver.right_stick_x, RobotConstants.CENTRO_MIN_POWER, RobotConstants.CENTRO_MAX_POWER);
        robot.motorCentro.setPower(motorCenterOutput);

        double controlPower = driver.left_stick_y * RobotConstants.ARMS_POWER_SCALE;
        robot.motorBraçoA.setPower(controlPower);
        robot.motorBraçoB.setPower(controlPower);

    }
}
