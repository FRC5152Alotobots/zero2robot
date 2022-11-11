package frc.robot.driveSys;



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Mecanum {
    // allow teleop and autonomous to use the same mecanum class
    private final WPI_VictorSPX frontLeft = new WPI_VictorSPX(3);
    private final WPI_VictorSPX rearLeft = new WPI_VictorSPX(4);
    private final WPI_VictorSPX frontRight = new WPI_VictorSPX(1);
    private final WPI_VictorSPX rearRight = new WPI_VictorSPX(2);
    private final MecanumDrive mecanumDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    private final Timer timer = new Timer();

    public void resetTimer() {
        timer.reset();
        timer.start();
        System.out.println("Timer reset");
    }
    public void mvMecanumTime(double x_speed, double y_speed, double z_speed, double game_time) {
        while (timer.get() < game_time) {
       mecanumDrive.driveCartesian(x_speed, y_speed, z_speed);
        }
        stopMecanum();
    }

    public void mvMecanum(double x_speed, double y_speed, double z_speed){
        rearRight.setInverted(true);
        if (x_speed > 1.0 || x_speed < -1.0 || y_speed > 1.0 || y_speed < -1.0 || z_speed > 1.0 || z_speed < -1.0){
            throw new IllegalArgumentException("Speeds must be between -1.0 and 1.0");
        } else {
            mecanumDrive.driveCartesian(x_speed*MecanumConstants.driveSpeedFactor, y_speed*MecanumConstants.driveSpeedFactor, z_speed*MecanumConstants.rotationSpeedFactor);

        }
    }

    public void stopMecanum() {
        mecanumDrive.stopMotor();
    }
    public void waitSeconds(double seconds) {
        try {
            Thread.sleep((long) (seconds * 1000));
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
