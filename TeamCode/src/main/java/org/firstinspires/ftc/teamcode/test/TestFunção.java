package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.test.TestPIDF.controller;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.DestemidosBot;

@Autonomous(name = "Teste: Função do Braço", group = "Test")
public class TestFunção extends LinearOpMode {

    double[] braçoPID = {TesteBraçoAutonomo.p, TesteBraçoAutonomo.i, TesteBraçoAutonomo.d, TesteBraçoAutonomo.f};

    public static void girarBraço(double angulo, DestemidosBot robot)
    {
        
        angulo = (TesteBraçoAutonomo.target);
        ConvertDegreesToTicks(angulo);
        angulo = target_in_ticks;

        motorBraçoACurrentPosition = robo.braçoA.getCurrentPosition();

        double pid = controller.calculate(motorBraçoACurrentPosition, target_in_ticks);
        double ff = Math.cos(Math.toRadians(target)) * f;

        double power = pid + ff; 

        robot.motorBraçoA.setPower(0); //TesteBraçoAutonomo.power);
        robot.motorBraçoB.setPower(0); //TesteBraçoAutonomo.power);

    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
