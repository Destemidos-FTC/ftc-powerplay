package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.UnitConversion;

public class SimpleArm implements Subsystem {

    // Motores
    public final DcMotorEx leftMotor;
    public final DcMotorEx rightMotor;

    // Controlador
    private PIDController controller;

    // Medidas do sistema
    private int target;
    private double pidPower;
    private double feedforwardPower;

    // Construtor padrão pra o mapeamento e iniciazlização dos componentes
    public SimpleArm(HardwareMap hardwareMap) {

        // mapeia os motores de acordo com os nomes no drivestatiom
        // TODO: defina a ID de cada motor
        leftMotor = hardwareMap.get(DcMotorEx.class, "");
        rightMotor = hardwareMap.get(DcMotorEx.class, "");

        // reseta os encoders
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // define a direção dos motores
        // TODO: checar a direção dos motores, se elas batem com as vistas no robô
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // desativa o PID interno do motor, que atualiza numa frequência mais lenta que o código
        // por isso, nós queremos ter o controle da "força pura" do motor
        // referência: https://www.ctrlaltftc.com/practical-examples/ftc-motor-control
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // define como o motor deve reagir quando chegar no "zero" de energia
        // nesse caso, queremos que ele desacelere imediatamente
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // definimos a tolerância da posição dos motores, na hora de ir até uma altura-alvo
        leftMotor.setTargetPositionTolerance(RobotConstants.ARM_POSITION_TOLERANCE);
        rightMotor.setTargetPositionTolerance(RobotConstants.ARM_POSITION_TOLERANCE);

        // inicializa o controlador PID com as váriavies editáveis do ftc-dashboard
        // usamos as váriáveis do RobotConstants.ARM_POSITION_PID
        controller = new PIDController(
                RobotConstants.ARM_POSITION_PID.p,
                RobotConstants.ARM_POSITION_PID.i,
                RobotConstants.ARM_POSITION_PID.d
        );
    }

    // Essa função fica sempre loopando durante a execução do robô
    // ela vai sempre checar pela posição do motor, e mandar pro controlador
    @Override
    public void periodic() {

        // pegamos a posição dos motores
        int leftPos = leftMotor.getCurrentPosition();
        int rightPos = rightMotor.getCurrentPosition();

        // vamos fazer uma média das posições, mas como a posição do motor é sempre um valor int
        // iremos "arredondar pra baixo", pra compensar uma possível diferença entre as medidas,
        // usando a função floor()
        double positionAverage = (leftPos + rightPos) / 2.0;
        int currentPosition = (int) Math.floor(positionAverage);

        // aplicamos os valores no pid, e calculamos a força necessária
        pidPower = controller.calculate(currentPosition, target);

        // calcula o feedforward (OPCIONAL)
        // referência: https://www.ctrlaltftc.com/feedforward-control#arm-feedforward

        // a função cosseno recebe em radianos
        // então já convertemos direto a posição alvo para essa media
        double armAngle = UnitConversion.encoderTicksToRadians(
                target, RobotConstants.CORE_HEX_TICKS_PER_REV);
        feedforwardPower = Math.cos(armAngle) * RobotConstants.ARM_POSITION_PID.f;

        // salvamos tudo em um comando
        double armCommand = pidPower; //+ feedforwardPower;

        // por fim, mandamos tudo de volta pros motores
        leftMotor.setTargetPosition(target);
        rightMotor.setTargetPosition(target);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(armCommand);
        rightMotor.setPower(armCommand);
    }

    public void setArmPosition(ArmSystem.ArmStage position) {
        switch (position) {
            case CLOSED:
                target = RobotConstants.ARM_CLOSED_GOAL;
                break;
            case LOW:
                target = RobotConstants.ARM_LOW_GOAL;
                break;
            case MEDIUM:
                target = RobotConstants.ARM_MEDIUM_GOAL;
                break;
            case HIGH:
                target = RobotConstants.ARM_HIGH_GOAL;
                break;
        }
    }

    public void moveArms(double joystick) {
        double controlPower = joystick * RobotConstants.FOREARM_POWER_SCALE;
        leftMotor.setPower(controlPower);
        rightMotor.setPower(controlPower);
    }
}
