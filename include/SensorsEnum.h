/**
 * @file include/JointsEnum.h
 *
 * This file declares the enumeration ids for all the sensors and 
 * actuators available in the simulated model of Nao
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jun 2017
 */

#pragma once

/**
 * Enumeration for available robot joints in simulation
 * 
 * @enum Joints
 */ 
enum Joints { 
	HEAD_YAW_POSITION = 0,
	HEAD_PITCH_POSITION,
	L_SHOULDER_PITCH_POSITION,
	L_SHOULDER_ROLL_POSITION,
	L_ELBOW_YAW_POSITION,
	L_ELBOW_ROLL_POSITION,
	L_WRIST_YAW_POSITION,
	//RHANDPOSITION,
	L_HIP_YAW_PITCH_POSITION,
	L_HIP_ROLL_POSITION,
	L_HIP_PITCH_POSITION,
	L_KNEE_PITCH_POSITION,
	L_ANKLE_PITCH_POSITION,
	L_ANKLE_ROLL_POSITION,
	R_HIP_YAW_PITCH_POSITION,
	R_HIP_ROLL_POSITION,
	R_HIP_PITCH_POSITION,
	R_KNEE_PITCH_POSITION,
	R_ANKLE_PITCH_POSITION,
	R_ANKLE_ROLL_POSITION,
	R_SHOULDER_PITCH_POSITION,
	R_SHOULDER_ROLL_POSITION,
	R_ELBOW_YAW_POSITION,
	R_ELBOW_ROLL_POSITION,
	R_WRIST_YAW_POSITION,
	NUM_JOINTS
};

/**
 * Enumeration for available robot cameras in simulation
 * 
 * @enum Joints
 */ 
enum Cameras { 
	TOP_CAM = 0,
	BOTTOM_CAM,
	NUM_CAMERAS
};

/**
 * Enumeration for available foot sensors in simulation
 * 
 * @enum Joints
 */ 
enum Fsr { 
	L_FOOT_FSR_FL = 0,
	L_FOOT_FSR_FR,
	L_FOOT_FSR_RL,
	L_FOOT_FSR_RR,
	R_FOOT_FSR_FL,
	R_FOOT_FSR_FR,
	R_FOOT_FSR_RL,
	R_FOOT_FSR_RR,
	NUM_FSR_SENSORS,
};
