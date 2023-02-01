package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.test.TestFunção.girarBraço;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.DestemidosBot;

import java.util.concurrent.TimeUnit;

@Config
@Autonomous(name = "Teste Braço Autonomo", group = "Test")
public class TesteBraçoAutonomo extends OpMode {
    private DestemidosBot robo;
    private PIDController controller;
    private ElapsedTime timer;
    private double tempoDecorrido;

    public static double p = 7, i = 0, d = 4.3;
    public static double f = 0.0015;

    // a posição alvo será medida em graus
    public static double target = 90;
    
    private double ConvertTicksToDegrees(int ticks) {
        return (double) ticks / 360.0;
    }

    private int ConvertDegreesToTicks(double degrees) {
        return (int) (1120 / (int) 360.0 * degrees) ;
    }

    @Override
    public void init() {
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        robo = new DestemidosBot(hardwareMap);
        robo.resetArmsEncoder();

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        double startTime = System.currentTimeMillis();

        // recebemos a posição do motor em ticks
        int motorBraçoACurrentPosition = robo.motorBraçoA.getCurrentPosition();
        int motorBraçoBCurrentPosition = robo.motorBraçoB.getCurrentPosition();

        // convertemos graus em ticks
        int target_in_ticks = ConvertDegreesToTicks(target);

        // nosso setpoint e o valor medido serão em ticks
        double pid = controller.calculate(motorBraçoACurrentPosition, target_in_ticks);
        double ff = Math.cos(Math.toRadians(target)) * f;

        double power = pid + ff;

        // definimos a posição alvo convertida
        robo.motorBraçoA.setTargetPosition(target_in_ticks);
        robo.motorBraçoB.setTargetPosition(target_in_ticks);

        // aplicamos a força calculada no PID, e dividimos para ambos motores
        robo.motorBraçoA.setPower(power);
        robo.motorBraçoB.setPower(power);

        // enviamos o comando pra os motores moverem posição
        robo.motorBraçoA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robo.motorBraçoB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

       if(motorBraçoACurrentPosition != target_in_ticks){
            // tempo decorrido tentando alcançar o setpoint
            tempoDecorrido = System.currentTimeMillis() - startTime;
       }

       telemetry.addData("BraçoA - Pos", motorBraçoACurrentPosition);
       telemetry.addData("BraçoB - Pos", motorBraçoBCurrentPosition);
       telemetry.addData("Target", target_in_ticks);
       telemetry.addData("forca do motorA", robo.motorBraçoA.getPower());
       telemetry.addData("forca do motorB", robo.motorBraçoB.getPower());
       telemetry.addData("forca do pid", power);
       telemetry.addData("valor do ff: ", ff);
       telemetry.addData("tempo até o alvo: ", tempoDecorrido);
    }
}