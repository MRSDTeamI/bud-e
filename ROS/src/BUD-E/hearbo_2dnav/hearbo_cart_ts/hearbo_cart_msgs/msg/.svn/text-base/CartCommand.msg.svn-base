#
# iXs cart control message
#

#
# Some pre-defined parameters
#

uint8 FALSE=0
uint8 TRUE=1

# ====================================================================================
# Message structure setting
# ====================================================================================

#
# valid : 
# 
# Major selection of message structure sent to the robot.
# You can set the following three commands valid/invalid.
# (1) Commands for drive (steering angle, velocity of driving wheels, selection of free/driven motors)
# (2) Gain of each motor
# (3) Motor selection for encoder value reset
# Those three settings do not have to be sent to the robot simultaneously.
#
# Example1
# If you want to only change the gain, set "gain_flag" TRUE.
#
# Example2
# If you want to change both velocity and gain of the motors, set "gain_flag" and "drive_flag" TRUE. 
# 

uint8 drive_flag
uint8 gain_flag
uint8 encoder_flag

# ====================================================================================
# Message when drive_flag is TRUE
# ====================================================================================

#
# drive_mode : cart control mode
#
# STOP_MODE : 
# This set all the gain zero (Encoder is not reset)
#
# TURN_MODE : 
# Spin rotation mode (Desired encoder values of steering motors is fixed for spinning)
#
# DRIVE_MODE : 
# Steering syncronization mode (Desired encoder values of steering motors are set to be the same.)
#
# MANUAL_MODE :
# You can set the following things freely for each motor.
# (1) Angle of steering motors, (3) Velocity of driving wheels
#
# RESTART_MODE : 
# This set all the gain default values (Encoder is not reset)
#

uint8 drive_mode
uint8 STOP_MODE=0
uint8 TURN_MODE=1
uint8 DRIVE_MODE=2
uint8 MANUAL_MODE=3
uint8 RESTART_MODE=4

#
# drive_id : selection of motor id (This decides which motor is processed/unprocessed.)
# ID0 : Left  front wheel 
# ID1 : Left  back  wheel 
# ID2 : Right back wheel 
# ID3 : Right front wheel 
# This setting is valid when the "drive_mode" is TURN_MODE/DRIVE_MODE/MANUAL_MODE.
# 
# Set each element of "drive_id" TRUE for activation and FALSE for deactivation.
# drive_id[0] : Left  front wheel 
# drive_id[1] : Left  back  wheel 
# drive_id[2] : Right back wheel 
# drive_id[3] : Right front wheel 
# If you set FALSE for some axes, the motors of the axes will be deactivated (free motors). 
#

uint8[4] drive_id

#
# each_vel : desired velocity of each drive wheel
# This setting is valid when the "drive_mode" is MANUAL_MODE.
# 
# Set each element of "each_vel" as the desired velocity [mm/s].
# each_vel[0] : Left  front wheel 
# each_vel[1] : Left  back  wheel 
# each_vel[2] : Right back wheel 
# each_vel[3] : Right front wheel 
#

float32[4] each_vel

#
# each_ang : desired angle of each steering motor
# This setting is valid when the "drive_mode" is MANUAL_MODE.
# 
# Set each element of "each_ang" as the desired angle [deg].
# each_ang[0] : Left  front wheel 
# each_ang[1] : Left  back  wheel 
# each_ang[2] : Right back wheel 
# each_ang[3] : Right front wheel 
#

float32[4] each_ang

#
# all_vel : desired velocity of all drive wheels
# This setting is valid when the "drive_mode" is TURN_MODE/DRIVE_MODE.
# 
# Set "all_vel" as the desired velocity [mm/s].
# Then the desired velocities of all driving wheels are set. 
#

float32 all_vel

#
# all_ang : desired angle of all steering motors
# This setting is valid when the "drive_mode" is DRIVE_MODE.
# 
# Set "all_ang" as the desired angle [deg].
# Then the desired angles of all steering motors are set. 
#

float32 all_ang

# ====================================================================================
# Message when gain_flag is TRUE
# ====================================================================================

#
# gain_id_ch : selection of motor id (This decides which gain is set/ignored.)
# 
# Set each element of "gain_id_ch0" and "gain_id_ch1" TRUE for activation and FALSE for deactivation.
# gain_id_ch0[0] : Left  front driving wheel 
# gain_id_ch0[1] : Left  back  driving wheel 
# gain_id_ch0[2] : Right back  driving wheel 
# gain_id_ch0[3] : Right front driving wheel 
# gain_id_ch1[0] : Left  front steering motor 
# gain_id_ch1[1] : Left  back  steering motor 
# gain_id_ch1[2] : Right back  steering motor 
# gain_id_ch1[3] : Right front steering motor 
#

uint8[4] gain_id_ch0
uint8[4] gain_id_ch1

#
# gain_ch : propotional gain setting for each motor (P control)
# 
# Set each element of "gain_ch" as the desired propotional gain.
# gain_ch0[0] : Left  front driving wheel 
# gain_ch0[1] : Left  back  driving wheel 
# gain_ch0[2] : Right back  driving wheel 
# gain_ch0[3] : Right front driving wheel 
# gain_ch1[0] : Left  front steering motor 
# gain_ch1[1] : Left  back  steering motor 
# gain_ch1[2] : Right back  steering motor 
# gain_ch1[3] : Right front steering motor 
#

int16[4] gain_ch0
int16[4] gain_ch1

# ====================================================================================
# Message when encoder_flag is TRUE
# ====================================================================================

#
# encoder_id_ch : selection of motor id (This decides which motor encoder is reset.)
# 
# Set each element of "encoder_id_ch0" and "encoder_id_ch1" TRUE for activation and FALSE for deactivation.
# encoder_id_ch0[0] : Left  front driving wheel 
# encoder_id_ch0[1] : Left  back  driving wheel 
# encoder_id_ch0[2] : Right back  driving wheel 
# encoder_id_ch0[3] : Right front driving wheel 
# encoder_id_ch1[0] : Left  front steering motor 
# encoder_id_ch1[1] : Left  back  steering motor 
# encoder_id_ch1[2] : Right back  steering motor 
# encoder_id_ch1[3] : Right front steering motor 
#

uint8[4] encoder_id_ch0
uint8[4] encoder_id_ch1

#
# encoder_ch : encoder reset value setting for each motor
# 
# Set each element of "encoder_ch" as the encoder value after reset.
# encoder_ch0[0] : Left  front driving wheel 
# encoder_ch0[1] : Left  back  driving wheel 
# encoder_ch0[2] : Right back  driving wheel 
# encoder_ch0[3] : Right front driving wheel 
# encoder_ch1[0] : Left  front steering motor 
# encoder_ch1[1] : Left  back  steering motor 
# encoder_ch1[2] : Right back  steering motor 
# encoder_ch1[3] : Right front steering motor 
#

int32[4] encoder_ch0
int32[4] encoder_ch1

