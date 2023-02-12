package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.DestemidosBot;

/**
 * RobotLogger - Responsável por transmitir informações dos
 * devidos dispositivos para o Telemetry, como forma de ajudar
 * a identificar possiveis problemas na hora de testatgem ou debug
 * dos OpModes.
 **/

public final class RobotLogger {

    public static void showSimpleMotorInfo(Telemetry telemetry, DcMotorEx motor, String name) {
        telemetry.addData("Motor name: ", name);
        telemetry.addData("Mode: ", motor.getMode());
        telemetry.addData("Power: ", motor.getPower());
        telemetry.addData("Position: ", motor.getCurrentPosition());
        telemetry.addData("Velocity", motor.getVelocity());
        telemetry.addData("Direction", motor.getDirection());
    }
    public static void showFullMotorInfo(Telemetry telemetry, DcMotorEx motor, String name) {
        telemetry.addData("Motor name: ", name);
        telemetry.addData("Port", motor.getPortNumber());
        telemetry.addData("Mode: ", motor.getMode());
        telemetry.addData("Power: ", motor.getPower());
        telemetry.addData("Position: ", motor.getCurrentPosition());
        telemetry.addData("Target Postion: ", motor.getTargetPosition());
        telemetry.addData("Current in Amps: ", motor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Velocity", motor.getVelocity());
        telemetry.addData("Direction", motor.getDirection());
        telemetry.addData("Coeficientes PIDF", motor.getPIDFCoefficients( motor.getMode() ));
    }

    public static void debugServoInfo(Servo servo, Telemetry telemetry) {
        telemetry.addData("Servo Name:", servo.getDeviceName());
        telemetry.addData("Servo Position:", servo.getPosition());
        telemetry.addData("Servo Direction:", servo.getDirection());
        telemetry.addData("Servo Port:", servo.getPortNumber());
    }

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
