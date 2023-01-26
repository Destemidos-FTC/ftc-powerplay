package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.DestemidosHardware;
@Config
@Autonomous(name = "Teste Braço Autonomo", group = "Test")
public class TesteBraçoAutonomo extends OpMode {
    private DestemidosHardware robo;
    private PIDController controller;

    public static double p = 0.175, i = 0, d = -0.2;
    public static double f = 0.3;

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
        robo = new DestemidosHardware(hardwareMap);
        robo.motorBraçoA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }
       
    @Override
    public void loop(){

        girarBraço(double angulo)

        // recebemos a posição do motor em ticks
        int motorBraçoACurrentPosition = robo.motorBraçoA.getCurrentPosition();
        int motorBraçoBCurrentPosition = robo.motorBraçoB.getCurrentPosition();

        // convertemos ticks em graus
        double braçoA_in_degree = ConvertTicksToDegrees(motorBraçoACurrentPosition);

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

        telemetry.addData("BraçoA - Pos", motorBraçoACurrentPosition);
        telemetry.addData("BraçoB - Pos", motorBraçoBCurrentPosition);
        telemetry.addData("Target", target_in_ticks);
        telemetry.addData("forca do pid", power);
        telemetry.addData("forca do motor", robo.motorBraçoA.getPower());
        telemetry.addData("valor do ff: ", ff);
        telemetry.update();
    }
}