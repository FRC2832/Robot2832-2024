This repository is designed to be a template library to start robot development with.  It also contains standard library functions the team has developed to help develop future robots.  It should always be updated to the latest WpiLib release.

Any code that is in the org.livoniawarriors folder is designed to be a library.  Ideally, nothing in that logic needs to change, and it can't have any configurations dependent on current robot code (aka from the frc.robot project).  In the future, this might become it's own project with a git subproject into a folder to load into.  Any configuring of the core should be done with persistent network tables so that the systems can be tuned.

See more documentation at [docs](src/main/java/org/livoniawarriors/docs/)

By default, files that are changed by the user (like configurations for the Sim GUI or AdvantageScope) are on gitignore so they won't get checked in, but users can get default versions of these in the defaultconfig/ folder.

Standard units:
* X axis = robot forward positive, back negative
* Y Axis = robot left positive, right negative
* Z axis = robot up positive, down negative
* Rotation = turn left positive (counter clockwise), right negative (clockwise)

Robot logging is on by default, we use standard [WpiLog](https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog.html) files from WpiLib.  To access the logs, insert a flash drive and they log automatically on there, or use Advantage Scope or the WpiLib Data Log Tool to get them off the robot.  The path should be /home/lvuser.  A standard SSH/SCP tool will work too, the username is lvuser, no password.

Notes:
We require both Phoenix libraries 5 and 6 for the robot.  5 is needed for older CTRE device support like the TalonSrx module.

We had to add a bunch of @SuppressWarnings("removal") to some of the legacy interfaces, as it looks like some of the CanSparkMax and TalonFX objects are changing for 2025.  It is do we want to use the new libraries, or use what we know works?