package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.DestemidosBot;
/*
 * RobotLogger - Responsável por transmitir informações dos
 * devidos dispositivos para o Telemetry, como forma de ajudar
 * a identificar possiveis problemas na hora de testatgem ou debug
 * dos OpModes.
 */

public final class RobotLogger {

    public static void showSimpleMotorInfo(Telemetry telemetry, DcMotorEx... motors) {
        for (DcMotorEx motor : motors) {
            telemetry.addData("Motor Info: ", "%s", motor.getDeviceName());
            telemetry.addData("Power: ", motor.getPower());
            telemetry.addData("Position: ", motor.getCurrentPosition());
        }
    }

    public static void showFulleMotorInfo(Telemetry telemetry, DcMotorEx... motors){
        for (DcMotorEx motor : motors) {
            showSimpleMotorInfo(telemetry, motor);
            telemetry.addData("Target Postion: ", motor.getTargetPosition());
            telemetry.addData("Current: ", motor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Velocity", motor.getVelocity());
            telemetry.addData("Direction", motor.getDirection());
            telemetry.addData("Port", motor.getPortNumber());
        }
    }

    // mostra informações úteis sobre as rodas
    public static void debugRodasInfo(Telemetry telemetry, DestemidosBot robot) {
        telemetry.addLine("\nMOTORES - Informações:");
        telemetry.addData("Power: ",
                "DF: %.2f  DT: %.2f \n EF: %.2f  ET: %.2f",
                robot.drivetrain.getMotorDireitaFrente().getPower(),
                robot.drivetrain.getMotorDireitaTras().getPower(),
                robot.drivetrain.getMotorEsquerdaFrente().getPower(),
                robot.drivetrain.getMotorEsquerdaTras().getPower()
        );

        telemetry.addData("Velocity: ",
                "DF: %.2f  DT: %.2f \n EF: %.2f  ET: %.2f",
                robot.drivetrain.getMotorDireitaFrente().getVelocity(AngleUnit.RADIANS),
                robot.drivetrain.getMotorDireitaTras().getVelocity(AngleUnit.RADIANS),
                robot.drivetrain.getMotorEsquerdaFrente().getVelocity(AngleUnit.RADIANS),
                robot.drivetrain.getMotorEsquerdaTras().getVelocity(AngleUnit.RADIANS)
        );

        telemetry.addData("Direction: \n",
                "- DF: %s \n DT: %s \n EF: %s \n ET: %s",
                robot.drivetrain.getMotorDireitaFrente().getDirection(),
                robot.drivetrain.getMotorDireitaTras().getDirection(),
                robot.drivetrain.getMotorEsquerdaFrente().getDirection(),
                robot.drivetrain.getMotorEsquerdaTras().getDirection()
        );
    }

    public static void debugServos(Telemetry telemetry, Servo... servos) {
        telemetry.addLine("SERVOS - Informações:");
        for (Servo s : servos) {
            telemetry.addData("Name: ", "%s", s.getDeviceName());
            telemetry.addData("- Position: ", s.getPosition());
            telemetry.addData("- Direction: ", s.getDirection());
            telemetry.addData("- Port: ", s.getPortNumber());
            telemetry.addLine("\n");
        }
    }

    // Invertemos o valor do Y, por causa que o SDK define
    // para cima como "-1.0", e para baixo como "+1.0"
    public static void debugControles(Telemetry telemetry, Gamepad controller1, Gamepad controller2) {
        telemetry.addLine("\nGAMEPADS: ");
        telemetry.addData("Gamepad1:",
                "Y: %.2f  X: %.2f  Giro: %.2f",
                controller1.left_stick_y,
                controller1.left_stick_x,
                controller1.right_stick_x
        );

        telemetry.addData("Gamepad2:",
                "Y: %.2f  X: %.2f  Giro: %.2f",
                controller2.left_stick_y,
                controller2.left_stick_x,
                controller2.right_stick_x
        );
    }
}
