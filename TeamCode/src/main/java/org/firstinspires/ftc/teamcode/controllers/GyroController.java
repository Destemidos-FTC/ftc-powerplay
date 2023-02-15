package org.firstinspires.ftc.teamcode.controllers;

/**
 * Controlador P dedicado a correção angular
 */
public class GyroController {

    private double kP;

    public double correctAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }

        // resultado em radiano
        return angle;
    }

    public GyroController(double kP) {
        this.kP = kP;
    }

    public double calculate(double targetAngle, double currentAngle) {
        // corrige a diferença de angulo, tudo em radiano
        double error = correctAngle(targetAngle - currentAngle);

        // TODO: inserir o termo F
        return (kP * error); // + (kF * 0);
    }
}
