package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class PathGroup extends CommandGroup{
	
	  /**
	   * Creates a new {@link CommandGroup CommandGroup}. The name of this command will be set to its
	   * class name.
	   */
	  public PathGroup() {
		  this.addSequential(new FollowPath(FollowPath.PATH_TYPE.TEST_PATH_CURVE));
		  this.addSequential(new FollowPath(FollowPath.PATH_TYPE.TEST_PATH_CURVE_REVERSE));
	  }

}
