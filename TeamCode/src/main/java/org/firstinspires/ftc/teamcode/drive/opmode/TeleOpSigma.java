package org.firstinspires.ftc.teamcode.drive.opmode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.A;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.B;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.C;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.D;

@TeleOp(group = "drive")
public class TeleOpSigma extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0,0,0));
        double forward = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        double heading;

        waitForStart();
        while(!isStopRequested()){
            drive.updatePoseEstimate();
            heading = (drive.getPoseEstimate().getHeading());
            if(heading<1){
                A.setPower(forward+(heading+90)*(1.0/45.0) + strafe + turn);
                B.setPower(forward+heading*(1.0/45.0) - strafe + turn);
                C.setPower(forward+(heading+90)*(1.0/45.0) + strafe - turn);
                D.setPower(forward+heading*(1.0/45.0) - strafe - turn);
            }
            else {
                A.setPower(forward - heading * (1.0 / 45.0) + strafe + turn);
                B.setPower(forward - (heading - 90) * (1.0 / 45.0) - strafe + turn);
                C.setPower(forward - heading * (1.0 / 45.0) + strafe - turn);
                D.setPower(forward - (heading - 90) * (1.0 / 45.0) - strafe - turn);
            }}}
}

