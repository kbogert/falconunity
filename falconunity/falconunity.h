//  FalconUnity
//
//  Created by Kyle Johnsen on 4/29/12.  With minor, inconsequential changes by Kenneth Bogert.  Superficial stuff really, forget I said anything.
//  Copyright (c) 2012 University of Georgia.
//
/*  
    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 3.0 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
    USA
*/
#ifndef falconunity_falconunity_h
#define falconunity_falconunity_h

struct _falconunity_spring_params {
	int spring_num;
	float max_force;
	float dampingFactor;
	float goalPos[3];
	float goalOrient[4];
	float posConstraintLower[3];
	float posConstraintUpper[3];
	float orientConstraintLower[3];
	float orientConstraintUpper[3];
	int directionality[6];
	float time_in_secs;

};

struct _falconunity_falcon_params {
	int falcon_num;
	float tipPositions[3];
	float godPositions[3];
	int buttons[4];
	float curforces[3];
};

struct _falconunity_object_params {
	int object_num;
	float pos[3];
	float orient[4];
};

struct _falconunity_haptic_tip_params {
	int falcon_num;
	float pos[3];
	float orient[4];
	float scale[3];
	bool useCompensator;
	float time_in_secs;

};

#ifdef _WIN32
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT 
#endif
extern "C" {
    DLLEXPORT int initFalconUnity(); //returns number of falcons
    DLLEXPORT bool closeFalconUnity(); //close all falcons
	DLLEXPORT float getFPS();


	DLLEXPORT bool getTipPosition(int falcon_num, float pos_out[3]);
	DLLEXPORT bool getGodPosition(int falcon_num, float pos_out[3]);
    DLLEXPORT bool getFalconButtonStates(int falcon_num,int buttons[4]);
	DLLEXPORT bool getFalconForces(int falcon_num, float force_out[3]);
    DLLEXPORT void getLastError(char * buffer); //pass the error out
    DLLEXPORT bool getCalibrated(int falcon_num);

	DLLEXPORT bool startLog(int falcon_num);
	DLLEXPORT bool stopLog(int falcon_num, int & num_entries, float * god_obj_positions, float * haptic_tip_positions, float * god_obj_forces, float * haptic_tip_forces, float * delta_ts);

	DLLEXPORT bool setForceField(int falcon_num, float force[3]); //set the falcon to send this constant force
    DLLEXPORT bool setSphereGodObject(int falcon_num, float radius, float mass, float pos[3], float minDistToMaxForce, float maxDistToMaxForce); //set end effector to a sphere    
    DLLEXPORT bool setRigidBodyGodObject(int falcon_num, int body_num, float minDistToMaxForce, float maxDistToMaxForce);
	DLLEXPORT bool removeGodObject(int falcon_num);
	DLLEXPORT bool updateHapticTransform(int falcon_num, float offset[3], float rot[4], float scales[3], bool useCompensator, float time_in_secs);
	
	DLLEXPORT bool setGravity(float force[3]);

    DLLEXPORT bool sendDynamicShape(int body_num, float * shape, int num_tris, float weight, float hardness, float startPos[3], float startOrient[4], float linearFactors[3], float angularFactors[3], float friction );
	DLLEXPORT bool removeDynamicShape(int body_num);
    DLLEXPORT bool updateDynamicShape(int body_num, float weight, float hardness, float linearFactors[3], float angularFactors[3], float friction );
	DLLEXPORT bool getDynamicShapePose(int body_num, float pos[3], float orient[4]);
	DLLEXPORT bool setDynamicShapePose(int body_num, float pos[3], float orient[4]);
	DLLEXPORT bool applyForceToShape(int body_num, float linear[3], float torque[3]);

	DLLEXPORT bool addSpringToShape(int body_num, int spring_num, float max_force, float dampingFactor, float goalPos[3], float goalOrient[4], float posConstraintLower[3], float posConstraintUpper[3], float orientConstraintLower[3], float orientConstraintUpper[3], int directionality[6]);
	DLLEXPORT bool removeSpring(int spring_num);
	DLLEXPORT bool lerpSpring(int spring_num, float max_force, float dampingFactor, float goalPos[3], float goalOrient[4], float posConstraintLower[3], float posConstraintUpper[3], float orientConstraintLower[3], float orientConstraintUpper[3], int directionality[6], float time_in_secs);
	DLLEXPORT bool setSpring(int spring_num, float max_force, float dampingFactor, float goalPos[3], float goalOrient[4], float posConstraintLower[3], float posConstraintUpper[3], float orientConstraintLower[3], float orientConstraintUpper[3], int directionality[6]);

	DLLEXPORT bool applyForce(int falcon_num, float force[3], float time_in_secs);
	
};

extern "C++" {

#include <vector>
using namespace std;

	DLLEXPORT bool falcon_update_all(std::vector<_falconunity_haptic_tip_params *> haptic_params, std::vector<_falconunity_spring_params *> spring_params, float * fps, std::vector<_falconunity_falcon_params *> * falcon_params, std::vector<_falconunity_object_params *> * object_params);

}

#endif
