cmake_minimum_required(VERSION 2.6)
PROJECT(  omnirobotPyrep )

INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )

ROBOCOMP_IDSL_TO_ICE( CommonBehavior CameraRGBDSimple CameraRGBDSimple CameraRGBDSimplePub GenericBase JoystickAdapter Laser LaserPub OmniRobot OmniRobotPub HumanToDSRPub CoppeliaUtils JointMotorPub JointMotor KinovaArmPub)

