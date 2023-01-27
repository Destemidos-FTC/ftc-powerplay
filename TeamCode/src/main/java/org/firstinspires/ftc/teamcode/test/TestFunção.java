package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.hardware.DestemidosBot;

public final class TestFunção {

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
}
