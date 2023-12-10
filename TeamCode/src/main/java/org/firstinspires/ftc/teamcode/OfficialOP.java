package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.classes.CupaCatinca;
import org.firstinspires.ftc.teamcode.classes.Slides;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp (name = "Doamne Miluieste", group = "Linear Opmode") // 🙏🙏
@Config
public class OfficialOP  extends LinearOpMode {

    boolean cupaEsteJos = true; // 0 daca pune, 1 daca aduna 😺

    int val = 1000;
    DcMotorEx intake; // motor intake


    double speedM = 0.85f;

    SampleMecanumDrive sasiu; // cod sasiu

    CupaCatinca catincaMamica; // servouri cupa

    Slides glisiere;

    @Override
    public void runOpMode()
    {

        sasiu = new SampleMecanumDrive(hardwareMap);
        catincaMamica = new CupaCatinca(hardwareMap);
        glisiere = new Slides(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        //Init

        catincaMamica.returnHome();
        if(isStopRequested())
            return;
        waitForStart();

        while(opModeIsActive()) {


            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.3;
            double rx = gamepad1.right_stick_x;


            sasiu.setWeightedDrivePower(new Pose2d(y,
                    -x * 0.3,
                    -rx), 0.7);


            /*double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.8;
            double rx = gamepad1.right_stick_x;

            sasiu.setWeightedDrivePower(new Pose2d(y * speedM, -x * 0.3 , -rx * speedM)); // sasiu


            sasiu.setWeightedDrivePower(new Pose2d(y * speedM,
                    -x * 0.3,
                    -rx * speedM));


            double x = gamepad1.left_stick_x * 1.3;

            */


            if(gamepad1.dpad_up)
                catincaMamica.aruncaAvion();

            // gamepad 2 😼😼

            // intake aduna 🤨

            if(gamepad2.right_trigger > 0)
            {
                intake.setPower(-0.7*val);
                catincaMamica.rollerAduna();
                catincaMamica.vomitaPixeli();
            }
            else
            {
                intake.setPower(0);
                catincaMamica.rollerOpreste();
                catincaMamica.stopCatinca();
            }

            //intake && cupa vomita pixel

            if(gamepad1.left_bumper) {
                intake.setPower(1);
                catincaMamica.rollerScoate();
                catincaMamica.adunaPixeli();
            }

            if (gamepad2.triangle) {
                catincaMamica.put();
                cupaEsteJos = false;
            }
            if (gamepad2.cross) {
                catincaMamica.returnHome();
                cupaEsteJos = true;
            }

            // glisiere

            if(gamepad2.dpad_up){
                glisiere.up(3);
            }
            if(gamepad2.dpad_left) {
                glisiere.up(2);
            }
            if(gamepad2.dpad_right) {
                glisiere.up(1);
            }
            if (gamepad2.dpad_down) {
                if(cupaEsteJos == false) {

                    catincaMamica.returnHome();
                    cupaEsteJos = true;
                    sleep(400);
                }
                glisiere.down();
            }
            if(gamepad2.left_stick_button){
                glisiere.up(4);
            }
            if(gamepad2.left_bumper){
                glisiere.up(5);
            }


            glisiere.update();

            // Telemetry

            if(cupaEsteJos)
                telemetry.addData("Cupa Catinca", "e acasa");
            else
                telemetry.addData("Cupa Catinca", "e plecata");
            telemetry.addData("upl", catincaMamica.upl.getPosition());
            telemetry.addData("upr", catincaMamica.upr.getPosition());
            // https://192.168.49.1:8080/dash
            telemetry.update();
        }
    }
}