# Pose logger extension

NOTE: Datalogger extension has to be disabled and enabled every time a new scene is loaded due to some trouble with the 
`World.instance()` not updating propperly

1. Load a scene with a mobile robot
2. Enable extension named "Pose logger" or `omni.isaac.pose_logger`
3. Initialize datalogger
4. Start simulation
5. Start logger
6. Save log
