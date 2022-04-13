package frc.robot.commands.Autons;

import org.ejml.ops.ConvertMatrixData;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.Autons.PathweaverAutons.FourBallAuto;
import frc.robot.commands.Autons.PathweaverAutons.GoonBall;
import frc.robot.commands.Autons.PathweaverAutons.MirroredTwoBallAuto;
import frc.robot.commands.Autons.PathweaverAutons.MoveForward;
import frc.robot.commands.Autons.PathweaverAutons.ThreeBallAuto;
import frc.robot.commands.Autons.PathweaverAutons.TrollTwoBall;
import frc.robot.commands.Autons.PathweaverAutons.TwoBallAuto;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DualRollerLauncher;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoChooser {

    public enum AutoMode{
        MOVE_FORWARD,
        TWO_BALL,
        MIRRORED_TWO_BALL,
        THREE_BALL,
        TROLL_TWO_BALL,
        GOON_BALL,
        FOUR_BALL
    }

    private SendableChooser<AutoMode> chooser;
    ShuffleboardTab coachTab = Shuffleboard.getTab("Coach Dashboard");
    ShuffleboardLayout auto =
      coachTab.getLayout("Auto Setup", "List Layout").withPosition(0, 0).withSize(1, 5);

    public AutoChooser(){
        chooser = new SendableChooser<AutoMode>();
        chooser.setDefaultOption("Two Ball", AutoMode.TWO_BALL);
        chooser.addOption("Mirrored Two Ball", AutoMode.MIRRORED_TWO_BALL);
        chooser.addOption("Three Ball", AutoMode.THREE_BALL);
        chooser.addOption("Four Ball", AutoMode.FOUR_BALL);
        chooser.addOption("Troll Ball (1)", AutoMode.TROLL_TWO_BALL);
        chooser.addOption("Goon Ball (2)", AutoMode.GOON_BALL);
        chooser.addOption("Move Forward (TEST ONLY)", AutoMode.MOVE_FORWARD);
        auto.add("Auto Chooser", chooser);
    }

    public void startMode(IntakeSubsystem intake, DualRollerLauncher drl, DriveTrainSubsystem drive, ConveyorSubsystem conveyor){
        AutoMode mode = (AutoMode)(chooser.getSelected());
        switch(mode){
            case MOVE_FORWARD:
                new MoveForward(intake, drl, drive, conveyor);
                break;
            case TWO_BALL:
                new TwoBallAuto(intake, drl, drive, conveyor);
                break;
            case MIRRORED_TWO_BALL:
                new MirroredTwoBallAuto(intake, drl, drive, conveyor);
                break;
            case THREE_BALL:
                new ThreeBallAuto(intake, drl, drive, conveyor);
                break;
            case FOUR_BALL:
                new FourBallAuto(intake, drl, drive, conveyor);
                break;
            case TROLL_TWO_BALL:
                new TrollTwoBall(intake, drl, drive, conveyor);
                break;
            case GOON_BALL:
                new GoonBall(intake, drl, drive, conveyor);
                break;
            default:
                break;
            
        }
    }
}
