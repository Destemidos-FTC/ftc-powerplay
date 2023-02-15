package org.firstinspires.ftc.teamcode.hardware;

import android.graphics.Color;

import androidx.annotation.NonNull;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.Arrays;
import java.util.List;

public final class DestemidosBot {

    // lista com todos os hubs e seus IDs para fácil acesso
    private final List<LynxModule> allHubs;
    private static final int CONTROLHUB_ID = 0;
    private static final int EXPANSIONHUB_ID = 0;

    // Drivetain
    public Drivetrain drivetrain;

    // Atuadores
    public DcMotorEx motorBraçoA;
    public DcMotorEx motorBraçoB;

    // Servos
    public Servo servoMão;
    public Servo servoGarraA;
    public Servo servoGarraB;

    // utilitário
    public List<DcMotorEx> atuadores;

    public DestemidosBot(@NonNull HardwareMap hardwareMap){

        // listando todos os hubs conectados no robô
        allHubs = hardwareMap.getAll(LynxModule.class);

        // uma frescurinha que descobri no dia do intersesi
        allHubs.get(CONTROLHUB_ID).setConstant(Color.CYAN);
        allHubs.get(EXPANSIONHUB_ID).setConstant(Color.CYAN);

        // carregando a configuração completa do drivetrain
        drivetrain = new Drivetrain(hardwareMap, Drivetrain.Mode.FULL);

        // configurando os atuadores dos braços
        motorBraçoA = hardwareMap.get(DcMotorEx.class,"braçoA"); // porta 1 - expansion
        motorBraçoB = hardwareMap.get(DcMotorEx.class,"braçoB"); // porta 2 - expansion
        
        motorBraçoA.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBraçoB.setDirection(DcMotorSimple.Direction.FORWARD);
        
        motorBraçoA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBraçoB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // configurando os servos da garra
        servoMão    = hardwareMap.get(Servo.class, "mão");    // porta 1 - controlhub
        servoGarraA = hardwareMap.get(Servo.class, "garraA"); // porta 3 - controlhub
        servoGarraB = hardwareMap.get(Servo.class, "garraB"); // porta 5 - controlhub
        
        servoMão.setDirection(Servo.Direction.REVERSE);
        servoGarraA.setDirection(Servo.Direction.REVERSE);
        servoGarraB.setDirection(Servo.Direction.FORWARD);

        // resetando todos os encoders do braço
//        resetArmsEncoder();

        // inicalizando a lista de atuadores já configurados
        atuadores = Arrays.asList(motorBraçoA, motorBraçoB);
    }

    // reinicia a contagem relativa dos encoders do braço
    public void resetArmsEncoder(){
        motorBraçoA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBraçoB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // ativa o modo de "caching" automático dos dados recebidos pelos hubs
    public void setBulkReadToAuto() {
        for (LynxModule robotHub : allHubs) {
            robotHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    // OBS: fica na resposabilidade do usuário, a limpeza desses dados em cache
    // se não tiver certeza de como realizar essa ação, recomendo o configurar no automático
    public void setBulkCacheToManual() {
        for (LynxModule robotHub : allHubs) {
            robotHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    };
}
