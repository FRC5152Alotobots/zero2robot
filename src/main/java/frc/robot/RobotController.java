package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.driveSys.Mecanum;

public class RobotController {

    /*------------------------------------------------------- [AUTONOMOUS] --------------------------------------------------*/
    private final Mecanum mecanum = new Mecanum();
    private final XboxController  joystick = new XboxController(0);

    /** This function is run once each time the robot enters autonomous mode. */

    public void autonomousInit() {
        mecanum.resetTimer();
    }

    /** This function is called periodically during autonomous. */
    public void autonomousPeriodic() {
        // Drive for 2 seconds
        mecanum.mvMecanumTime(1.0, 0.0, 0.0, 3.0); // drive forwards half speed until 3 sec
        mecanum.waitSeconds(1.0); // wait 1 sec
        mecanum.mvMecanumTime(-1.0, 0.0, 0.0, 7.0); // drive backwards half speed until game time reaches 7 sec (first drive 3sec+1sec wait+3 sec new drive = 7 sec)

    }


/*---------------------------------------------- [TELEOP] ---------------------------------------------------------- */
        /** This function is called once each time the robot enters teleoperated mode. */
        public void teleopInit() {
            // send a message to the driver station
            System.out.println("Teleop mode started");
        }
    
        /** This function is called periodically during teleoperated mode. */
        public void teleopPeriodic() {
            if (){
                
            }
            // drive the robot
            mecanum.mvMecanum(joystick.getLeftY()*-1/2, joystick.getLeftX()/2, joystick.getRightX()/2);
        }
        
}
