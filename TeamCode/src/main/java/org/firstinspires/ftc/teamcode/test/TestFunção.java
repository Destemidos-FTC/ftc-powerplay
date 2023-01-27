package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.hardware.DestemidosBot;

public final class TestFunção {

    double[] braçoPID = {TesteBraçoAutonomo.p, TesteBraçoAutonomo.i, TesteBraçoAutonomo.d, TesteBraçoAutonomo.f};

    public static void girarBraço(double angulo, DestemidosBot robot)
    {
        
        robot.motorBraçoA.setPower(0); //TesteBraçoAutonomo.power);
        robot.motorBraçoB.setPower(0); //TesteBraçoAutonomo.power);

    }
}
