package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.test.TestPIDF.controller;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.hardware.DestemidosBot;

@Autonomous(name = "Teste: Função do Braço", group = "Test")
public class TestFunção extends LinearOpMode {
    private DestemidosBot robot;
    private PIDFController ArmController;

    int ConvertDegreesToTicks2(double degrees) {
        return (int) (1120 / (int) 360.0 * degrees) ;
    }

    // coeficientes e váriáveis modificáveis
    public static double p = 0.175, i = 0, d = -0.2, f = 0.3;
    public static double targetAngle;

    private void girarBraço(double anguloAlvo, DcMotorEx armMotor)
    {
        // recebemos a posição atual e o alvo do motor
        int target_in_ticks = ConvertDegreesToTicks2(anguloAlvo);
        int current_arm_pos =  armMotor.getCurrentPosition();

        // carregamos as constantes no pid;
        ArmController.setPIDF(p, i, d, f);

        // calculamos a ação do pid e o impulso do feedforward
        double pidOutput = ArmController.calculate(current_arm_pos, target_in_ticks);
        double feedforwardAngle = Math.cos(Math.toRadians(anguloAlvo)) * f;

        // enviamos a posição alvo pro motor
        armMotor.setTargetPosition(target_in_ticks);

        // e entregamos a força somada do pid e do feedforward
        double powerToMotor = pidOutput + feedforwardAngle;
        armMotor.setPower(powerToMotor);

        // finalizando com o comando de seguir até a posição
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new DestemidosBot(hardwareMap);
        ArmController = new PIDFController(p,i, d, f);

        waitForStart();
        while (opModeIsActive()) {

            girarBraço(targetAngle, robot.motorBraçoA);
        }
    }
}
