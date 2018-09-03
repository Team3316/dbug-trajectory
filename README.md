dbug-trajectory
===

> D-Bug's trajectory planning utility for the 2019 Deep Space season

Since the 2014 Aerial Assist season, the need for autonomous path planning in FRC has rose and reached its peak during the 2018 Power Up season,
where in order to be able to control most elements of the field during autonomous teams had to use some sort of path planning. Until 2018, we used simple PID distance and
angle control in order to achieve straight line driving and turning, and during 2018 we used the wonderful [SmoothPathPlanner](https://github.com/KHEngineering/SmoothPathPlanner)
utility by @KHEngineering. After encountering some adjustments we wanted to make to the tool and the time it took to calculate trajectories mid-game, we decided
to roll up our sleeves this season and build our own tool for trajectory planning.

After the 2018 season, we designed this tool with the following principles in mind:
 - Calculating the trajectories should be fast and require minimal data
 - Accounting for the physical constraints of the robot
 - Minimizing the computing power needed from the RoboRIO during the game
 - Outputting data to use in feed-forward control alongside feed-back control
 - Ease of use and fast computation time

With all of this in mind, we decided to develop a standalone tool that will be operated on a laptop rather on the RoboRIO in order to save as much 
computation time and power usage during the game.

**Note - This tool is still very much WIP, so bugs might occur here and there. If you encounter a bug, please open an issue!** 