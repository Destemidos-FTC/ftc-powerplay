package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

@Deprecated
public class Outtake extends SubsystemBase {

    // Hardware devices
    public final DcMotorEx leftOuttake;
    public final DcMotorEx rightOuttake;

    // Controladores
    private final PIDController outtakeController;
    public static double p = 0, i = 0, d = 0;

    private double leftPos = 0;
    private double rightPos = 0;

    private int targetPos;

    public ExtensionStage currentStage = ExtensionStage.ZERO;

    public enum ExtensionStage {
        ZERO,
        LOW,
        MEDIUM,
        HIGH
    }

    public Outtake(HardwareMap hardwareMap) {
        leftOuttake = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftOuttake");
        rightOuttake = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightOuttake");
        leftOuttake.setDirection(DcMotorSimple.Direction.REVERSE);

        outtakeController = new PIDController(
                RobotConstants.OUTTAKE_PID.p,
                RobotConstants.OUTTAKE_PID.i,
                RobotConstants.OUTTAKE_PID.d);

        rightOuttake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOuttake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // inicializando
        targetPos = 0;
    }

    @Override
    public void periodic() {

        // carregamos as constantes modificáveis do ftc-dashboard
        outtakeController.setPID(
                RobotConstants.OUTTAKE_PID.p,
                RobotConstants.OUTTAKE_PID.i,
                RobotConstants.OUTTAKE_PID.d);


        leftPos = leftOuttake.getCurrentPosition();
        rightPos = rightOuttake.getCurrentPosition();

        double pidOutput = outtakeController.calculate(leftPos, targetPos);
        double power = pidOutput;

        leftOuttake.setPower(power);
        rightOuttake.setPower(power);
    }

    public void setGoal(ExtensionStage goalStage) {

        // salvamos em qual estágio o intake tá
        currentStage = goalStage;

        switch (goalStage) {
            case ZERO:
                targetPos = 0;
                break;

            case LOW:
                targetPos = 500;
                break;

            case MEDIUM:
                targetPos = 1500;
                break;

            case HIGH:
                targetPos = 3000;
                break;
        }
    }

    public ExtensionStage getGoal() {
        return  currentStage;
    }
}
