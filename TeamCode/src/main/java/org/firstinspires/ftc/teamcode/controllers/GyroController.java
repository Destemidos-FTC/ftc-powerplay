package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.utils.MathUtils;

/**
 * PID Dedicado ao giroscópio, com base no guia da CTRL ALT FTC:
 * referências: https://www.ctrlaltftc.com/the-pid-controller
 */
public class GyroController {

    private double kP;
    private double kI;
    private double kD;
    private double kF;

    // P
    private double error;
    private double lastError = 0;
    private double lastAngle = 0;

    // I
    private double integralSum = 0;
    private double minIntegralSum = -1.0;
    private double maxIntegralSum = 1.0;

    // D
    private double errorChange = 0;
    private double currentFilterEstimate = 0;
    private double lastFilterEstimate = 0;
    public double filterGain;

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

    public GyroController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public GyroController(PIDFCoefficients coefficients) {
        this.kP = coefficients.p;
        this.kI = coefficients.i;
        this.kD = coefficients.d;
        this.kF = coefficients.f;
    }

    public void setPID(double kp, double ki, double kd) {
        this.kP = kp;
        this.kI = ki;
        this.kD = kd;
    }

    public double calculate(double targetAngle, double currentAngle) {

        double currentTime = (System.nanoTime() / 1E9);

        // corrige a diferença de angulo, tudo em radiano
        error = correctAngle(targetAngle - currentAngle);

        // aplicando o filtro na derivada
        errorChange = error - lastError;

        //currentFilterEstimate = (filterGain * lastFilterEstimate) + (1 - filterGain) * errorChange;
        //lastFilterEstimate = currentFilterEstimate;
        //double derivative = currentFilterEstimate / currentTime;

        double derivative = errorChange / currentTime;

        // limitando a soma da integral
        integralSum += (error * currentTime);
        integralSum = MathUtils.Clamp(integralSum, minIntegralSum, maxIntegralSum);

        // reseta a integral quando o angulo alvo muda
        if(targetAngle != lastAngle) {
            integralSum = 0;
        }

        lastError = error;
        lastAngle = targetAngle;

        // TODO: inserir o termo F
        return (kP * error) + (kI * integralSum) + (kD * derivative) + (kF * 0);
    }
}
