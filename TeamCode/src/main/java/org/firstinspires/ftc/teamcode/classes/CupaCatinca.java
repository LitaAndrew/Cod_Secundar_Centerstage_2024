package org.firstinspires.ftc.teamcode.classes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.apache.commons.math3.analysis.integration.IterativeLegendreGaussIntegrator;


public class CupaCatinca {

    public CRServo servoIntake;
    public CRServo servoCupaCatinca;

    public ServoImplEx upl = null;

    public ServoImplEx upr = null;

    public ServoImplEx avion = null;

    public CupaCatinca(@NonNull HardwareMap hardwareMap)
    {
        upl = hardwareMap.get(ServoImplEx.class, "upl");
        upr = hardwareMap.get(ServoImplEx.class, "upr");
        servoCupaCatinca = hardwareMap.get(CRServo.class, "w");
        servoIntake = hardwareMap.get(CRServo.class, "servoIntake");
        avion = hardwareMap.get(ServoImplEx.class, "avion");

        upr.setDirection(ServoImplEx.Direction.REVERSE);
    }

    public void put()
    {
        upl.setPosition(0.37);
        upr.setPosition(0.37);
    }
    public void returnHome()
    {
        upl.setPosition(0.085);
        upr.setPosition(0.085);
    }
    public void adunaPixeli()
    {
        servoCupaCatinca.setPower(1);
    }
    public void vomitaPixeli()
    {
        servoCupaCatinca.setPower(-1);
    }
    public void stopCatinca()
    {
        servoCupaCatinca.setPower(0);
    }

    public void rollerAduna()
    {
        servoIntake.setPower(1);
    }
    public void rollerScoate()
    {
        servoIntake.setPower(-1);
    }
    public void rollerOpreste()
    {
        servoIntake.setPower(0);
    }
    public void aruncaAvion()
    {
        avion.setPosition(0.5);
    }
}
