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

//all includes
#include <iostream>
#include <vector> 
#include "falconunity.h"
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <map>
#include <boost/array.hpp>
#include <boost/thread.hpp> 
#include <boost/timer/timer.hpp>
#include <btBulletDynamicsCommon.h>
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "hr_time.h"

#ifndef M_PI
#define M_PI 3.14159265359
#endif

//mac includes
#ifndef _WIN32 

#include "falcon/core/FalconLogger.h"
#include "falcon/core/FalconDevice.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/grip/FalconGripFourButton.h"
#include "falcon/kinematic/FalconKinematicStamper.h"

using namespace libnifalcon;
#else
#include <Windows.h>
#include <hdl/hdl.h>
#include <hdlu/hdlu.h>
#endif

#define MAX_FALCON_FORCE 9

using namespace std;
//for errors
char last_error[1024]="none";


//but there should only be one world
btDefaultCollisionConfiguration* collisionConfiguration;
btCollisionDispatcher* dispatcher;
btBroadphaseInterface*	broadphase;
vector<btRigidBody*> rigidBodies;
btSequentialImpulseConstraintSolver* solver;
map<int, btRigidBody *> sentRigidBodies;
btDiscreteDynamicsWorld* dynamicsWorld;
boost::recursive_mutex collision_mutex ;

#define TOTAL_FRAMES 5
float frame_times[TOTAL_FRAMES];
int next_frame;

class MyKinematicMotionState : public btMotionState {
public:
    MyKinematicMotionState(const btTransform &initialpos) { mPos1 = initialpos; }
    virtual ~ MyKinematicMotionState() { }
    virtual void getWorldTransform(btTransform &worldTrans) const { /*cout << "Sending transform" << endl;*/ worldTrans = mPos1; }
    void setKinematicPos(btTransform &currentPos) { mPos1 = currentPos; }
    virtual void setWorldTransform(const btTransform &worldTrans) { }

protected:
    btTransform mPos1;
};


#ifndef _WIN32 
vector<FalconDevice *>falcons;
boost::shared_ptr<FalconFirmware> ff;
#else
HDLOpHandle servo_op;
#endif


class FalconInterface {

private:
	boost::recursive_mutex * mutex ;

	HDLDeviceHandle falconHandle;
	btRigidBody * godObject;
	bool godObjectIsSphere;

	btVector3 * scale;
	btTransform * transform;
	float maxForce;
	float minDistToMaxForceInMeters;
	float maxDistToMaxForceInMeters;
	btVector3 * constantForce;
	
	bool collideFlag;
	float collideObjRestitution;

	btVector3 * curTipPosition;

	int buttonMask;


	btVector3 * applyForceVector;
	float applyForceTimeLeft;
	
	bool compensatorActivated;


	btVector3 * savedForces[TOTAL_FRAMES];
	int curForceNum;

	// for lerping

	btVector3 * posLerpStart;
	btVector3 * posLerpEnd;
	btVector3 * scaleLerpStart;
	btVector3 * scaleLerpEnd;
	btQuaternion * rotationLerpStart;
	btQuaternion * rotationLerpEnd;

	float lerpTotalTime;
	float lerpElapsedTime;

	btScalar collideObjMass;
	btScalar collideObjImpulse;

public:
	FalconInterface(HDLDeviceHandle falcon, float maxForceInNewtons) {
		falconHandle = falcon;

		godObject = 0;
		godObjectIsSphere = false;
		scale = new btVector3(1,1,1);
		transform = new btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0));
		curTipPosition = new btVector3(0,0,0);
		constantForce = new btVector3(0,0,0);

		maxForce = maxForceInNewtons;
		mutex = new boost::recursive_mutex();
		collideFlag = false;

		applyForceVector = NULL;
		applyForceTimeLeft = 0;

		posLerpStart = NULL;
		posLerpEnd = NULL;
		rotationLerpStart = NULL;
		rotationLerpEnd = NULL;
		scaleLerpStart = NULL;
		scaleLerpEnd = NULL;

		lerpTotalTime = 0;
		lerpElapsedTime = 0;

		compensatorActivated = false;
		curForceNum = 0;
		for (int i = 0; i < TOTAL_FRAMES; i ++) {
			savedForces[i] = new btVector3(0,0,0);
		}

		collideObjMass = 1;
		collideObjImpulse = 0;

	}

	void setGodObject (btRigidBody * godObject, float minToMaxForce, float maxToMaxForce, bool isSphere) {
	    boost::recursive_mutex::scoped_lock lock2( *mutex ) ;

		this->godObject = godObject;

//		minDistToMaxForceInMeters = .05; // need to vary this with the scale
//		maxDistToMaxForceInMeters = .15; // this too

		minDistToMaxForceInMeters = minToMaxForce;
		maxDistToMaxForceInMeters = maxToMaxForce;

		godObjectIsSphere = isSphere;
	}

	bool isGodObjectSphere() {
		return godObjectIsSphere;
	}

	~FalconInterface() {
		clearLerpParams();
		for (int i = 0; i < TOTAL_FRAMES; i ++) {
			delete savedForces[i];
		}
		if (applyForceVector != NULL)
			delete applyForceVector;
		delete scale;
		delete transform;
		delete constantForce;
		delete curTipPosition;
		delete mutex;
		
	}

	void update(float deltaT) {
	    boost::recursive_mutex::scoped_lock lock_it( collision_mutex ) ;
		boost::recursive_mutex::scoped_lock lock2( *mutex ) ;

		// get lastest haptic tip position
		hdlMakeCurrent(falconHandle);
		boost::array<double,3> incomming;
		hdlToolPosition(incomming.c_array());

		btVector3 newPos = btVector3(incomming[0], incomming[1], incomming[2]);


		// are we lerping?  if so, call setParams with the lerped values
		if (lerpTotalTime >= lerpElapsedTime && lerpTotalTime != 0) {

			lerpElapsedTime += deltaT;

			float t = lerpElapsedTime / lerpTotalTime;
			t = min(t, 1.0f);

			btTransform newTf(rotationLerpStart->slerp(*rotationLerpEnd, t), posLerpStart->lerp(*posLerpEnd, t));

			if (! collideFlag && godObject != 0 && compensatorActivated) {
				// apply compensator to the god object to prevent the haptic tip from feeling the forces of moving the transform

				godObject->setWorldTransform(btTransform(godObject->getWorldTransform().getRotation(), newTf * transform->invXform(godObject->getWorldTransform().getOrigin())));

			}
			setTransform(newTf, scaleLerpStart->lerp(*scaleLerpEnd, t));

		}

		newPos[0] *= (*scale)[0];
		newPos[1] *= (*scale)[1];
		newPos[2] *= (*scale)[2];
		
		newPos = ((*transform) * newPos);

		btVector3 hapticForce(0, 0, 0);

		// have we set the godObject? If not, skip over the spring calcs
		if (godObject != 0) {
		
			
			// get god object position, convert to haptics world coordinates
			btVector3 godObjPos = godObject->getWorldTransform().getOrigin();

			// calc distance between haptic tip and god object
			btVector3 distance = godObjPos - newPos;

			// if collided, use restitution from collision object
			// if not colided, decay restitution
			// calc spring constant: k
			if (! collideFlag) {
				collideObjRestitution += .1f * (1 - collideObjRestitution);
			}
		
			float distToMaxForce = collideObjRestitution * maxDistToMaxForceInMeters;

			if (distToMaxForce == 0)
				distToMaxForce = minDistToMaxForceInMeters * .00001;

			float k = maxForce / distToMaxForce;
		
			// calculate spring forces
			btVector3 springForce = -k * distance;

			// use mass and k to calc damping force: c
			float c;
			
			if (godObject->getInvMass() == 0) { 
				// Godobject is static, use movement of haptic tip to calc damping forces

				c = 2 * sqrt( k * .003f );
				btVector3 x_prime = ((*curTipPosition) - newPos) / deltaT;

				springForce += -c * x_prime; // add damping force based on velocity of haptic tip to the spring forces

			} else {
				
				c= 2 * sqrt( k / godObject->getInvMass() );

				btVector3 dampingForce = -c * godObject->getLinearVelocity();;


				// calc total force and apply to god object
				btVector3 totalForce = springForce + dampingForce;
				if (applyForceTimeLeft > 0) {
					totalForce += *applyForceVector;
					applyForceTimeLeft -= deltaT;
				}
				
				if (collideObjMass == 0 && collideFlag) {
					totalForce = distance.normalized() * collideObjImpulse /1000.0f;
				}


				// apply to God Object
				godObject->activate();
				godObject->applyCentralForce(totalForce);


			}


			// send spring force to haptic device

			if (scale->getX() < 0)
				springForce[0] *= -1;
			if (scale->getY() < 0)
				springForce[1] *= -1;
			if (scale->getZ() < 0)
				springForce[2] *= -1;


			hapticForce += btTransform(transform->getRotation(), btVector3(0,0,0)) * springForce;
		}


		hapticForce += *constantForce;
		hapticForce *= -1;

		boost::array<double,3> to_send;
		to_send[0] = hapticForce[0];
		to_send[1] = hapticForce[1];
		to_send[2] = hapticForce[2];

		hdlSetToolForce(to_send.c_array());

		// clear colision flag
		collideFlag = false;

		// download button states
		hdlToolButtons(&buttonMask);

		curTipPosition->setValue(newPos[0], newPos[1], newPos[2]);

		curForceNum ++;
		curForceNum = curForceNum % TOTAL_FRAMES;
		savedForces[curForceNum]->setValue(hapticForce[0], hapticForce[1], hapticForce[2]);

	}

	void setHasCollided(float restitution, btScalar objectMass, btScalar appliedImpulse) {
		collideFlag = true;
		collideObjRestitution = 1.0f - restitution;
		this->collideObjMass = objectMass;
		this->collideObjImpulse = appliedImpulse;
	}

	btVector3 getPosition() {
	    boost::recursive_mutex::scoped_lock lock_it( *mutex ) ;

		return * curTipPosition;
	}

	btRigidBody * getGodObject() {
		return godObject;
	}

	HDLDeviceHandle getHDLHandle() {
		return falconHandle;
	}

	btVector3 getAveragedForces() {
	    boost::recursive_mutex::scoped_lock lock_it( *mutex ) ;
		btVector3 sum(0,0,0);
		for (int i = 0; i < TOTAL_FRAMES; i ++) {
			sum += *(savedForces[i]);
		}
		return sum / TOTAL_FRAMES;
	}

	void setConstantForce(const btVector3 & force) {
	    boost::recursive_mutex::scoped_lock lock_it( *mutex ) ;

		delete constantForce;
		constantForce = new btVector3(force);
	}

	bool getButtonState(int num) {
	    boost::recursive_mutex::scoped_lock lock_it( *mutex ) ;

		int i = 1 << num;
		return (buttonMask & i) > 0;
	}

	void applyForce(const btVector3 & force, float time_in_secs) {
	    boost::recursive_mutex::scoped_lock lock_it( *mutex ) ;
		if (applyForceVector != NULL)
			delete applyForceVector;
		
		applyForceVector = new btVector3(force);
		applyForceTimeLeft = time_in_secs;
	}

	void setTransform(const btTransform & trans, const btVector3 & scale) {
	    boost::recursive_mutex::scoped_lock lock_it( *mutex ) ;

		delete transform;
		delete this->scale;

		transform = new btTransform(trans);
		this->scale = new btVector3(scale);
	}

	void setTransform(const btTransform & trans, const btVector3 & scale, bool useGodObjectCompensator, float lerpTime) {
	    boost::recursive_mutex::scoped_lock lock_it( *mutex ) ;

		compensatorActivated = useGodObjectCompensator;

		clearLerpParams();
		if (lerpTime <= 0) {
			
			if (! collideFlag && godObject != 0 && compensatorActivated) {
				// apply compensator to the god object to prevent the haptic tip from feeling the forces of moving the transform

				godObject->setWorldTransform(btTransform(godObject->getWorldTransform().getRotation(), trans * transform->invXform(godObject->getWorldTransform().getOrigin())));

			}

			setTransform(trans, scale);
			return;
		}

		posLerpStart = new btVector3(transform->getOrigin());
		rotationLerpStart = new btQuaternion(transform->getRotation());
		scaleLerpStart = new btVector3(*this->scale);

		posLerpEnd = new btVector3(trans.getOrigin());
		rotationLerpEnd = new btQuaternion(trans.getRotation());
		scaleLerpEnd = new btVector3(scale);

		lerpTotalTime = lerpTime;
		lerpElapsedTime = 0;

	}

	void clearLerpParams() {
		boost::recursive_mutex::scoped_lock lock_it( *mutex ) ;

		if (posLerpStart != NULL)
			delete posLerpStart;

		if (posLerpEnd != NULL)
			delete posLerpEnd;

		if (rotationLerpStart != NULL) 
			delete rotationLerpStart;

		if (rotationLerpEnd != NULL)
			delete rotationLerpEnd;

		if (scaleLerpStart != NULL)
			delete scaleLerpStart;

		if (scaleLerpEnd != NULL)
			delete scaleLerpEnd;


		posLerpStart = NULL;
		posLerpEnd = NULL;
		rotationLerpStart = NULL;
		rotationLerpEnd = NULL;
		scaleLerpStart = NULL;
		scaleLerpEnd = NULL;

		lerpTotalTime = 0;
		lerpElapsedTime = 0;
		
	}
};
/*
void QuaternionToEulerXYZ(const btQuaternion &quat,btVector3 &euler)
{
   float w=quat.getW();   float x=quat.getX();   float y=quat.getY();   float z=quat.getZ();
   double sqx = x*x; double sqy = y*y; double sqz = z*z; 
/*   euler.setZ((atan2(2.0 * (x*y + z*w),(sqx - sqy - sqz + sqw))));
   euler.setX((atan2(2.0 * (y*z + x*w),(-sqx - sqy + sqz + sqw))));
   euler.setY((asin(-2.0 * (x*z - y*w)))); */
/*
   euler.setX((atan2(2.0 * (w*x + y*z),1 - 2 * (sqx + sqy))));
   euler.setY((asin(2.0 * (w*y - z*x))));
   euler.setZ((atan2(2.0 * (w*z + x*y),1 - 2 * (sqy + sqz))));
}

void EulerXYZToQuaternion(btVector3 &euler, btQuaternion &quat)
{   
   btMatrix3x3 mat;
   mat.setIdentity();
   mat.setEulerZYX(euler.getX(), euler.getY(), euler.getZ());
   mat.getRotation(quat);

   //equivalent?
   //btQuaternion q(euler.getX(), euler.getY(), euler.getZ());
   //btQuaternion q1(q.getY(), q.getX(), q.getZ(), q.getW());
   //quat = q1;
}
*/

class DampedSpring {
private:
	boost::recursive_mutex * mutex ;

	btRigidBody * obj;
	btVector3 * springPos;
	btVector3 * posConstraintLower;
	btVector3 * posConstraintUpper;
	btQuaternion * springOrient;
	btVector3 * orientConstraintLower;
	btVector3 * orientConstraintUpper;
	float maxForce;
	float dampingFactor;
	int springDirectionalityCodes[6];


	// for lerping

	btVector3 * springPosLerpStart;
	btVector3 * posConstraintLowerLerpStart;
	btVector3 * posConstraintUpperLerpStart;
	btQuaternion * springOrientLerpStart;
	btVector3 * orientConstraintLowerLerpStart;
	btVector3 * orientConstraintUpperLerpStart;
	float maxForceLerpStart;
	float dampingFactorLerpStart;

	btVector3 * springPosLerpEnd;
	btVector3 * posConstraintLowerLerpEnd;
	btVector3 * posConstraintUpperLerpEnd;
	btQuaternion * springOrientLerpEnd;
	btVector3 * orientConstraintLowerLerpEnd;
	btVector3 * orientConstraintUpperLerpEnd;
	float maxForceLerpEnd;
	float dampingFactorLerpEnd;


	float lerpTotalTime;
	float lerpElapsedTime;


public:

	DampedSpring(btRigidBody * b, float force, float dFactor, const btVector3 & goalPos, const btQuaternion & goalOrient, const btVector3 & pConstl, const btVector3 & pConstu, const btVector3 & oConstl, const btVector3 & oConstu, const int * springDirectionality) {
		obj = b;
		springPos = new btVector3(goalPos);
		posConstraintLower = new btVector3(pConstl);
		posConstraintUpper = new btVector3(pConstu);
		springOrient = new btQuaternion(goalOrient);
		orientConstraintLower = new btVector3(oConstl);
		orientConstraintUpper = new btVector3(oConstu);
		maxForce = force;
		dampingFactor = dFactor;
		if (springDirectionality != 0) {
			for (int i = 0; i < 6; i ++) {
				springDirectionalityCodes[i] = springDirectionality[i];
			}
		} else {
			for (int i = 0; i < 6; i ++) {
				springDirectionalityCodes[i] = 0;
			}
		}

		mutex = new boost::recursive_mutex();

		lerpTotalTime = 0;
		lerpElapsedTime = 0;
		maxForceLerpStart = 0;
		maxForceLerpEnd = 0;
		dampingFactorLerpStart = 0;
		dampingFactorLerpEnd = 0;
		springPosLerpStart = NULL;
		posConstraintLowerLerpStart = NULL;
		posConstraintUpperLerpStart = NULL;
		springOrientLerpStart = NULL;
		orientConstraintLowerLerpStart = NULL;
		orientConstraintUpperLerpStart = NULL;
		springPosLerpEnd = NULL;
		posConstraintLowerLerpEnd = NULL;
		posConstraintUpperLerpEnd = NULL;
		springOrientLerpEnd = NULL;
		orientConstraintLowerLerpEnd = NULL;
		orientConstraintUpperLerpEnd = NULL;
	}

	~DampedSpring() {

		clearLerpParams();

		delete springPos;
		delete posConstraintLower;
		delete posConstraintUpper;
		delete springOrient;
		delete orientConstraintLower;
		delete orientConstraintUpper;
		delete mutex;
	}

	void update(float deltaT) {
		boost::recursive_mutex::scoped_lock lock_it( collision_mutex ) ;
		boost::recursive_mutex::scoped_lock lock2( *mutex ) ;

		
		// are we lerping?  if so, call setParams with the lerped values
		if (lerpTotalTime >= lerpElapsedTime && lerpTotalTime != 0) {

			lerpElapsedTime += deltaT;

			float t = lerpElapsedTime / lerpTotalTime;
			t = min(t, 1.0f);

			float f = (maxForceLerpEnd - maxForceLerpStart) * t + maxForceLerpStart;
			float d = (dampingFactorLerpEnd - dampingFactorLerpStart) * t + dampingFactorLerpStart;
			
			setParams(f, d, springPosLerpStart->lerp(*springPosLerpEnd, t), springOrientLerpStart->slerp(*springOrientLerpEnd, t), posConstraintLowerLerpStart->lerp(*posConstraintLowerLerpEnd, t), posConstraintUpperLerpStart->lerp(*posConstraintUpperLerpEnd, t), orientConstraintLowerLerpStart->lerp(*orientConstraintLowerLerpEnd, t), orientConstraintUpperLerpStart->lerp(*orientConstraintUpperLerpEnd, t), &(springDirectionalityCodes[0]) );

		}

		// calculate linear forces from spring model

		btVector3 curPos = obj->getWorldTransform().getOrigin();

		// calc distance between current position and goal object
		btVector3 distance = curPos - *springPos;

		bool reset = false;
		btVector3 v = obj->getLinearVelocity();
		for (int i = 0; i < 3; i ++) {
			if ((*posConstraintLower)[i] < 0) {
				if (distance[i] < (*posConstraintLower)[i]) {

					distance[i] = (*posConstraintLower)[i];
					reset = true;
					v[i] = 0;

				}
			}

			if ((*posConstraintUpper)[i] > 0) {
				if (distance[i] > (*posConstraintUpper)[i]) {

					distance[i] = (*posConstraintUpper)[i];
					reset = true;
					v[i] = 0;

				}
			}
		}
		if (reset) {
			btTransform trans(obj->getWorldTransform().getRotation(), btVector3(*springPos + distance));
			obj->setWorldTransform(trans);
			obj->setLinearVelocity(v);
			curPos = *springPos + distance;
		}


		float k = maxForce;

		// calculate spring forces
		btVector3 springForce = -k * distance;

		// are there restrictions on this spring? if so, check if they apply per dimension
		for (int i = 0; i < 3; i ++) {
			switch (springDirectionalityCodes[i]) {
			case -1:
				if (springForce[i] < 0)
					springForce[i] = 0;
				break;
			case 1:
				if (springForce[i] > 0)
					springForce[i] = 0;
				break;
			}
		}

		// use mass and k to calc damping force: c
		float c = dampingFactor * 2 * sqrt( k / obj->getInvMass() );
		btVector3 dampingForce = -c * v;


		// calc total force and apply to god object
		btVector3 totalForce = springForce + dampingForce;

		// if the directionality is 2 then we dont' want any forces at all on this dimension
		for (int i = 0; i < 3; i ++) {
			if (springDirectionalityCodes[i] == 2)
				totalForce[i] = 0;
		}

		// apply to Object
		obj->activate();
		obj->applyCentralForce(totalForce);



		// calculate rotational forces from spring model

		btQuaternion curRotation = obj->getWorldTransform().getRotation();

		// calc distance between current position and goal object

		v = obj->getAngularVelocity();
		
		// Multiply the current rotation times the inverse of the last one to get the amount rotated, then convert this to euler angles and add to sumRotation
		/*
		btVector3 eulerRotation;
		QuaternionToEulerXYZ(curRotation * (*lastRotation).inverse(), eulerRotation);

		(*sumRotation) += eulerRotation;

		// Due to loss of precision, over time sumRotation will be off.  Therefore if sumRotation < the actual rotation then use the actual rotation

		btVector3 actualRotation;
		QuaternionToEulerXYZ(curRotation * (*springOrient).inverse(), actualRotation);

		for (int i = 0; i < 3; i ++) {
			if (abs((*sumRotation)[i]) <  abs(actualRotation[i])) {
				(*sumRotation)[i] = actualRotation[i];
			}
		}

		*/
		
		// what I actually need here is the distance rotated from the goal in each dimension. hmmmm
		// TODO: So to do this:
		//	Every time we find a constraint violation, find the difference in euler rotation
		//	Build a quaternion out of this difference
		//  rotation correction by this quaternion, it will represent the sum of violated constraint corrections
		//  after we're done, multiply the object's rotation by the correction

		// now that we're doing this with axisangle, it's the same story but don't use sum rotation 
		btQuaternion correction(0,0,0,1);
		btVector3 rotDegree = curRotation.getAngle() * curRotation.getAxis();

		reset = false;
		for (int i = 0; i < 3; i ++) {
			if ((*orientConstraintLower)[i] < 0) {
				if (rotDegree[i] < (*orientConstraintLower)[i]) {

					btVector3 cor(0,0,0);
					cor[i] = 1;
					btScalar corAmt = (*orientConstraintLower)[i] - rotDegree[i];

					btQuaternion tempQuat(cor,corAmt);
					correction = tempQuat * correction;

					reset = true;
					v[i] = 0;

				}
			}

			if ((*orientConstraintUpper)[i] > 0) {
				if (rotDegree[i] > (*orientConstraintUpper)[i]) {

					btVector3 cor(0,0,0);
					cor[i] = 1;
					btScalar corAmt = (*orientConstraintUpper)[i] - rotDegree[i];

					btQuaternion tempQuat(cor, corAmt);
					correction = tempQuat * correction;

					reset = true;
					v[i] = 0;

				}
			}
		}
		if (reset) {
			btTransform trans(correction * obj->getWorldTransform().getRotation(), obj->getWorldTransform().getOrigin());
			obj->setWorldTransform(trans);
			obj->setAngularVelocity(v);
		}
		
		
		k = maxForce;

		// calculate spring forces
//		springForce = -k * (*sumRotation);

		// calculations with pure quaternions
		btQuaternion torqueQuat = (*springOrient).nearest(curRotation) * (*springOrient).inverse() ;
		springForce = torqueQuat.getAxis() * (-k * torqueQuat.getAngle());

		/*		static int counter = 0;
		counter ++;
		if (counter % 50 == 0) {
			cout <<curRotation.getX() << ", " << curRotation.getY() << ", " << curRotation.getZ() << ", " << curRotation.getW() << " -> " << (*springOrient).getX() << ", " << (*springOrient).getY() << ", " << (*springOrient).getZ() << ", " << (*springOrient).getW() << "\n";
			cout << "  : " << torqueQuat.getAxis().getX() << ", " << torqueQuat.getAxis().getY() << ", " << torqueQuat.getAxis().getZ() << " % " << torqueQuat.getAngle() << "\n";
		} */


		// are there restrictions on the direction of the spring?  If so, check the cardinality of the spring force's directions to see if they've been violated
		// if so, zero out that dimension's forces and set the velocity for that dimension to zero
		
		for (int i = 0; i < 3; i ++) {
			switch (springDirectionalityCodes[i+3]) {
			case -1:
				if (springForce[i] < 0)
					springForce[i] = 0;
				break;
			case 1:
				if (springForce[i] > 0)
					springForce[i] = 0;
				break;
			}
		}


		// use mass and k to calc damping force: c
		c= dampingFactor * 2 * sqrt( k / obj->getInvMass() );


		dampingForce = -c * v;

		// calc total force and apply to god object
		totalForce = springForce + dampingForce;
		
		// if the directionality is 2 then we dont' want any forces at all on this dimension
		for (int i = 0; i < 3; i ++) {
			if (springDirectionalityCodes[i + 3] == 2)
				totalForce[i] = 0;
		}

//		if (counter % 50 == 0) { cout << "  Force:   " << totalForce.getX() << ", " << totalForce.getY() << ", " << totalForce.getZ() << "\n";	}

		// apply to Object
		obj->activate();
		btVector3 min;
		btVector3 max;
		obj->getAabb(min, max);
		obj->applyTorque(totalForce * (max - min).length());

		// save current dist
//		lastRotation->setValue(curRotation[0], curRotation[1], curRotation[2], curRotation[3]);

	}

	void setParams(float force, float dampingFactor, const btVector3 & goalPos, const btQuaternion & goalOrient, const btVector3 & pConstl, const btVector3 & pConstu, const btVector3 & oConstl, const btVector3 & oConstu, const int * springDirectionality) {
		boost::recursive_mutex::scoped_lock lock2( *mutex ) ;

		maxForce = force;
		this->dampingFactor = dampingFactor;
		springPos->setValue(goalPos[0], goalPos[1], goalPos[2]);
		posConstraintLower->setValue(pConstl[0], pConstl[1], pConstl[2]);
		posConstraintUpper->setValue(pConstu[0], pConstu[1], pConstu[2]);

		if (springDirectionality != 0) {
			for (int i = 0; i < 6; i ++) {
				springDirectionalityCodes[i] = springDirectionality[i];
			}
		} else {
			for (int i = 0; i < 6; i ++) {
				springDirectionalityCodes[i] = 0;
			}
		}


//		btVector3 diff;
		// compute shortest rotation between previous goal orientation and the new one
		// add this to the sumRotation value
//		QuaternionToEulerXYZ(goalOrient * (*springOrient).inverse(), diff);
//		(*sumRotation) += diff;

		springOrient->setValue(goalOrient[0], goalOrient[1], goalOrient[2], goalOrient[3]);
		orientConstraintLower->setValue(oConstl[0], oConstl[1], oConstl[2]);
		orientConstraintUpper->setValue(oConstu[0], oConstu[1], oConstu[2]);
	}

	void clearLerpParams() {
		boost::recursive_mutex::scoped_lock lock2( *mutex ) ;

		if (springPosLerpStart != NULL)
			delete springPosLerpStart;

		if (posConstraintLowerLerpStart != NULL)
			delete posConstraintLowerLerpStart;

		if (posConstraintUpperLerpStart != NULL) 
			delete posConstraintUpperLerpStart;

		if (springOrientLerpStart != NULL)
			delete springOrientLerpStart;

		if (orientConstraintLowerLerpStart != NULL)
			delete orientConstraintLowerLerpStart;

		if (orientConstraintUpperLerpStart != NULL)
			delete orientConstraintUpperLerpStart;

		if (springPosLerpEnd != NULL)
			delete springPosLerpEnd;

		if (posConstraintLowerLerpEnd != NULL)
			delete posConstraintLowerLerpEnd;

		if (posConstraintUpperLerpEnd != NULL)
			delete posConstraintUpperLerpEnd;

		if (springOrientLerpEnd != NULL) 
			delete springOrientLerpEnd;

		if (orientConstraintLowerLerpEnd != NULL)
			delete orientConstraintLowerLerpEnd;

		if (orientConstraintUpperLerpEnd != NULL)
			delete orientConstraintUpperLerpEnd;


		springPosLerpStart = NULL;
		posConstraintLowerLerpStart = NULL;
		posConstraintUpperLerpStart = NULL;
		springOrientLerpStart = NULL;
		orientConstraintLowerLerpStart = NULL;
		orientConstraintUpperLerpStart = NULL;
		springPosLerpEnd = NULL;
		posConstraintLowerLerpEnd = NULL;
		posConstraintUpperLerpEnd = NULL;
		springOrientLerpEnd = NULL;
		orientConstraintLowerLerpEnd = NULL;
		orientConstraintUpperLerpEnd = NULL;
		
		lerpTotalTime = 0;
		lerpElapsedTime = 0;
		maxForceLerpStart = 0;
		maxForceLerpEnd = 0;
		dampingFactorLerpStart = 0;
		dampingFactorLerpEnd = 0;

	}

	void lerpParams(float force, float dampingFactor, const btVector3 & goalPos, const btQuaternion & goalOrient, const btVector3 & pConstl, const btVector3 & pConstu, const btVector3 & oConstl, const btVector3 & oConstu, const int * springDirectionality, float time_in_secs) {
		// save the current params, the provided ones, and the time to lerp
		boost::recursive_mutex::scoped_lock lock2( *mutex ) ;
		
		clearLerpParams();
		if (time_in_secs == 0) {
			setParams(force, dampingFactor, goalPos, goalOrient, pConstl, pConstu, oConstl, oConstu, springDirectionality);
			return;
		}

		if (springDirectionality != 0) {
			for (int i = 0; i < 6; i ++) {
				springDirectionalityCodes[i] = springDirectionality[i];
			}
		} else {
			for (int i = 0; i < 6; i ++) {
				springDirectionalityCodes[i] = 0;
			}
		}


		springPosLerpStart = new btVector3(*springPos);
		posConstraintLowerLerpStart = new btVector3(*posConstraintLower);
		posConstraintUpperLerpStart = new btVector3(*posConstraintUpper);
		springOrientLerpStart = new btQuaternion(*springOrient);
		orientConstraintLowerLerpStart = new btVector3(*orientConstraintLower);
		orientConstraintUpperLerpStart = new btVector3(*orientConstraintUpper);
		
		springPosLerpEnd = new btVector3(goalPos);
		posConstraintLowerLerpEnd = new btVector3(pConstl);
		posConstraintUpperLerpEnd = new btVector3(pConstu);
		springOrientLerpEnd = new btQuaternion(goalOrient);
		orientConstraintLowerLerpEnd = new btVector3(oConstl);
		orientConstraintUpperLerpEnd = new btVector3(oConstu);

		maxForceLerpStart = maxForce;
		maxForceLerpEnd = force;

		dampingFactorLerpStart = this->dampingFactor;
		dampingFactorLerpEnd = dampingFactor;

		lerpTotalTime = time_in_secs;
		lerpElapsedTime = 0;
	}

};

//falcon info is architecture independent
vector<FalconInterface *> falconInterfaces;

map<int, DampedSpring *> springs;

bool done=true;
boost::thread handle_thread;

#ifdef _WIN32
bool testHDLError();
HDLServoOpExitCode updateHDL(void* pUserData);
#endif
void updateHaptics();
void set_error(string error);
bool initCollisions();
int initFalcons();
bool check_falcon_num(int n);
bool check_falcon_num(int n){
    return (n>=0 && n<falconInterfaces.size());
}

#ifndef _WIN32 
void updateHaptics(){
    
    while(true){
        if(done){
            return;
        }
        for(int i=0;i<falcons.size();i++){
            
            FalconDevice * dev = falcons[i];
            if(dev->runIOLoop()){
            
            if(falconInfo[i].homing){
                    dev->getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::RED_LED);
                if(dev->getFalconFirmware()->isHomed()){
                    dev->getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::GREEN_LED);
                    falconInfo[i].homing = false;
                }
                
            }
            else{
                moveAndGenerateForces(i);
            }
            }
            
        }
    }
    
}
#else

HDLServoOpExitCode updateHDL(void* pUserData){
	static CStopWatch stopWatch;
	
	stopWatch.stopTimer();
	
	float elapsedTime = (float)stopWatch.getElapsedTime();
	stopWatch.startTimer();

	frame_times[next_frame] = elapsedTime;
	next_frame = (next_frame + 1) % TOTAL_FRAMES;

//	if (elapsedTime > 0.005f) {
//		printf("TimeBank: %f\n", elapsedTime);
//		elapsedTime = 0.005f;
//	}

	boost::recursive_mutex::scoped_lock lock_it( collision_mutex ) ;
//	dynamicsWorld->stepSimulation(elapsedTime, 20, 1.0f/2000.0f);
	dynamicsWorld->stepSimulation(elapsedTime, 10, 1.0f/1000.0f);
//	dynamicsWorld->stepSimulation(elapsedTime, 0); // sigh, this would be nice
	lock_it.unlock();

	for(unsigned int i=0;i<falconInterfaces.size();i++){
		falconInterfaces[i]->update(elapsedTime);
	}

	for (std::map<int,DampedSpring *>::iterator it=springs.begin(); it!=springs.end(); ++it) {
		it->second->update(elapsedTime);
	}


	return HDL_SERVOOP_CONTINUE;
}


void updateHaptics(){

	
	if (falconInterfaces.size() <= 0) {

		while(true) {
			if (done) {
				return;
			}

			Sleep(1);

			updateHDL(0);
		}

	} else {


		//create a servo op
		servo_op = hdlCreateServoOp(updateHDL,
									 (void*)0,
									 false);
		if(!testHDLError()){
			set_error("error creating servo op");
			return;
		}
		while(true){
			if(done){
				return;
			}
			Sleep(100);
		} 

	}

}
#endif
#ifdef _WIN32
bool testHDLError()
{
    HDLError err = HDL_NO_ERROR;
    err = hdlGetError();
    if (err != HDL_NO_ERROR)
    {
        return false;
    }
	return true;
}
#endif
void set_error(string error){
    sprintf(last_error, "%s", error.c_str());
}

void collisionCheckTickCallback(btDynamicsWorld *world, btScalar timeStep) {
	int numManifolds = world->getDispatcher()->getNumManifolds();
	for (int i=0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold =  world->getDispatcher()->getManifoldByIndexInternal(i);
		const btCollisionObject* obA = static_cast<const btCollisionObject*>(contactManifold->getBody0());
		const btCollisionObject* obB = static_cast<const btCollisionObject*>(contactManifold->getBody1());

		int found = -1;
		float rest = 0;
		btScalar mass;
		for (unsigned int k = 0; k < falconInterfaces.size(); k ++) {
			btCollisionObject* test = falconInterfaces[k]->getGodObject();
			if (test == obA) {
				found = k;
				rest = obB->getRestitution();
				mass = obB->isStaticOrKinematicObject () ? 0 : 1;
				break;
			} else if (test == obB) {
				found = k;
				rest = obA->getRestitution();
				mass = obA->isStaticOrKinematicObject () ? 0 : 1;
				break;
			} 
		}
		if (found < 0)
			continue;
	
		int numContacts = contactManifold->getNumContacts();
		for (int j=0;j<numContacts;j++)
		{
			btManifoldPoint& pt = contactManifold->getContactPoint(j);
			if (pt.getDistance()<=0.f)
			{
				falconInterfaces[found]->setHasCollided(rest, mass, pt.getAppliedImpulse());
				break;
			}
		}
	}

}


bool initCollisions(){
    
    boost::recursive_mutex::scoped_lock lock_it( collision_mutex ) ;
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
    btVector3	worldAabbMin(-1000,-1000,-1000);
    btVector3	worldAabbMax(1000,1000,1000);
    broadphase = new btDbvtBroadphase();

	solver = new btSequentialImpulseConstraintSolver();
 
    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
    dynamicsWorld->setInternalTickCallback(collisionCheckTickCallback);
    dynamicsWorld->setGravity(btVector3(0,0,0));

	btCollisionDispatcher * dispatcher = static_cast<btCollisionDispatcher *>(dynamicsWorld ->getDispatcher());
	btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);
    
	return true;
}

#ifndef _WIN32
int initFalcons(){

    //create one falcon
    FalconDevice *dev = new FalconDevice();
	dev->setFalconFirmware<FalconFirmwareNovintSDK>();
	ff = dev->getFalconFirmware();
    unsigned int num_falcons = 0;
	if(!dev->getDeviceCount(num_falcons))
	{
		set_error("Cannot get device count");
		return -1;
	}
    if(num_falcons == 0)
	{
		set_error("No falcons found");
		return 0;
	}
    
    delete dev; //delete the device (we'll make it again)
    
    //create all falcons and initialize them
    for(int i = 0; i < num_falcons; ++i)
	{
        FalconDevice * dev = new FalconDevice();
		falcons.push_back(dev);
        falconInfo.push_back(FalconInfo());
        
        
		dev->setFalconFirmware<FalconFirmwareNovintSDK>();
        dev->setFalconKinematic<libnifalcon::FalconKinematicStamper>();
        dev->setFalconGrip<libnifalcon::FalconGripFourButton>();
		if(!dev->open(i))
		{
			set_error("Cannot open falcon - Error: ");
			return -1;
		}
	}	
    
    //load all falcon's firmware
    for(int i = 0; i < num_falcons; ++i)
	{
		if(!falcons[i]->isFirmwareLoaded())
		{
            for(int z = 0; z < 10; ++z) //not sure why this is here
			{
                              
				if(!falcons[i]->getFalconFirmware()->loadFirmware(true, NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE)))
				{
					set_error("Could not load firmware");
					//return -1;
                    falcons[i]->runIOLoop();
				}
				else
				{
					break;
				}
                 
			}
			if(!falcons[i]->isFirmwareLoaded())
			{
				set_error("Firmware didn't load correctly. Try running findfalcons again");
				return -1;
			}
            
		}
	}
    
    for(int i=0;i< num_falcons; i++){
        
        falcons[i]->getFalconFirmware()->setHomingMode(true);
        falconInfo[i].homing = true;
        
    }
    return num_falcons;
}
#else



int initFalcons(){
	HDLError err = HDL_NO_ERROR;

	int num_falcons = hdlCountDevices();

	for(int i=0;i<num_falcons;i++){
		HDLDeviceHandle id = hdlInitIndexedDevice(i);
		if(!testHDLError()){
			set_error("error initializing falcon");
			return -1;
		}
		falconInterfaces.push_back(new FalconInterface(id, MAX_FALCON_FORCE));
	}

	if (num_falcons > 0) {
		hdlStart();


		//setup workspace dimensions
		double dims[6] = {0.0};
		for(int i=0;i<num_falcons;i++){
			hdlMakeCurrent(falconInterfaces[i]->getHDLHandle());
			hdlDeviceWorkspace(dims);
		}
	}
	return num_falcons;
}
    

#endif

bool closeFalconUnity(){
    if(!done){
        done = true;
        handle_thread.join();
        
#ifdef _WIN32 

		if (servo_op != 0)
			//destroy the servo op
			hdlDestroyServoOp(servo_op);

		if (falconInterfaces.size() > 0)
			//stop hdl
			hdlStop();

#endif
        for(unsigned int i=0;i<falconInterfaces.size();i++){
#ifndef _WIN32
            falcons[i]->close();
            delete falcons[i];
#else
			hdlUninitDevice(falconInterfaces[i]->getHDLHandle());
#endif
        }
        //
        
        //remove the rigidbodies from the dynamics world and delete them
   		for(std::vector<btRigidBody*>::iterator it = rigidBodies.begin(); it != rigidBodies.end(); ++it) {
			btRigidBody* rb = (btRigidBody*)*it;
			dynamicsWorld->removeRigidBody(rb);
			delete rb->getMotionState();
			delete rb->getCollisionShape();
			delete rb;
		}

		// delete springed objects
		for (std::map<int,DampedSpring *>::iterator it=springs.begin(); it!=springs.end(); ++it) {
			delete it->second;
		}
		springs.clear();

        //delete dynamics world
        delete dynamicsWorld;
        
        //delete broadphase
        delete broadphase;
        
        //delete dispatcher
        delete dispatcher;

		delete solver;
        
        delete collisionConfiguration;

		rigidBodies.clear();
		sentRigidBodies.clear();
		for (unsigned int i = 0; i < falconInterfaces.size(); i ++) {
			delete falconInterfaces[i];
		}
		falconInterfaces.clear();


    }
    return true;
}
int initFalconUnity(){
    if(!done){
        closeFalconUnity();
    }
	next_frame = 0;
    if(!initCollisions()){
        return false;
    }
	done = false;
    int num_falcons = initFalcons();
    //start the thread
    handle_thread = boost::thread(updateHaptics);
    return num_falcons;
}


bool getTipPosition(int falcon_num, float pos_out[3]){
    if(!check_falcon_num(falcon_num)){set_error("bad falcon number"); return false;}

	btVector3 p = falconInterfaces[falcon_num]->getPosition();
    for(int i=0;i<3;i++){
        pos_out[i] = (float)p[i];
    }
	
    return true;
}

bool getGodPosition(int falcon_num, float pos_out[3]){
    if(!check_falcon_num(falcon_num)){set_error("bad falcon number"); return false;}

	boost::recursive_mutex::scoped_lock lock_it( collision_mutex ) ;
	btRigidBody * go = falconInterfaces[falcon_num]->getGodObject();
	if (go == 0) {
		for(int i=0;i<3;i++){
			pos_out[i] = 0;
		}
		set_error("Falcon does not have a God Object set (call setSphereGodObject())");
		return false;
	}
	btVector3 p = go->getWorldTransform().getOrigin();
    for(int i=0;i<3;i++){
        pos_out[i] = (float)p[i];
    }
    return true;
}

bool getFalconForces(int falcon_num, float force_out[3]){
    if(!check_falcon_num(falcon_num)){set_error("bad falcon number"); return false;}

	btVector3 p = falconInterfaces[falcon_num]->getAveragedForces();
    for(int i=0;i<3;i++){
        force_out[i] = (float)p[i];
    }
	
    return true;
}

bool setForceField(int falcon_num, float force[3]){
    if(!check_falcon_num(falcon_num)){set_error("bad falcon number"); return false;}

	btVector3 f(force[0], force[1], force[2]);
	falconInterfaces[falcon_num]->setConstantForce(f);
    return true;
}
bool applyForce(int falcon_num, float force[3], float time_in_secs){
    if(!check_falcon_num(falcon_num)){set_error("bad falcon number"); return false;}
	
	btVector3 f(force[0], force[1], force[2]);
	falconInterfaces[falcon_num]->applyForce(f, time_in_secs);
    return true;
}

void getLastError(char * buffer){ //pass the error out
    
    sprintf(buffer, "%s", last_error);
    
}

bool getFalconButtonStates(int falcon_num, int buttons[4]){
    if(!check_falcon_num(falcon_num)){set_error("bad falcon number"); return false;}

#ifndef _WIN32
    for(int i=0;i<4;i++){
        buttons[i] = falcons[falcon_num]->getFalconGrip()->getDigitalInput(i)?1:0;
    }  

#else
	buttons[0] = falconInterfaces[falcon_num]->getButtonState(0);
	buttons[1] = falconInterfaces[falcon_num]->getButtonState(1);
	buttons[2] = falconInterfaces[falcon_num]->getButtonState(2);
	buttons[3] = falconInterfaces[falcon_num]->getButtonState(3);
#endif

    return true;
}

void removeSphereGodObject(int falcon_num) {
	btRigidBody * oldRigidBody = falconInterfaces[falcon_num]->getGodObject();

	if (oldRigidBody != NULL) {
		for(unsigned int i = 0;i< rigidBodies.size();i++){
			if(oldRigidBody == rigidBodies[i]){
				rigidBodies.erase(rigidBodies.begin()+i);
				break;
			}
		}
		//delete from bullet
		dynamicsWorld->removeRigidBody(oldRigidBody);
		//delete actual data
		delete oldRigidBody->getMotionState();
		delete oldRigidBody->getCollisionShape();
		delete oldRigidBody;
		
	}
}

bool setSphereGodObject(int falcon_num, float radius, float mass, float pos[3], float minDistToMaxForce, float maxDistToMaxForce) {        
    boost::recursive_mutex::scoped_lock lock_it( collision_mutex ) ;    

    if(!check_falcon_num(falcon_num)){set_error("bad falcon number"); return false;}
    
	if (falconInterfaces[falcon_num]->isGodObjectSphere())
		removeSphereGodObject(falcon_num);

	btCollisionShape * sh = new btSphereShape(radius);
    sh->setMargin(0.f); 

    btDefaultMotionState* motionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(pos[0],pos[1],pos[2])));
    btVector3 inertia(0,0,0);

    sh->calculateLocalInertia(mass,inertia);
    btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(mass,motionState,sh,inertia);
    
	btRigidBody* rigidBody = new btRigidBody(rigidBodyCI);
	rigidBody->setRestitution(.5);
	rigidBody->setAngularFactor(0);
	rigidBody->setDamping(0, 0);
//	rigidBody->setFriction(0);
	dynamicsWorld->addRigidBody(rigidBody);

	rigidBodies.push_back(rigidBody);

	falconInterfaces[falcon_num]->setGodObject(rigidBody, minDistToMaxForce, maxDistToMaxForce, true);
	return true;
}


bool setRigidBodyGodObject(int falcon_num, int body_num, float minDistToMaxForce, float maxDistToMaxForce) {        
    boost::recursive_mutex::scoped_lock lock_it( collision_mutex ) ;    

    if(!check_falcon_num(falcon_num)){set_error("bad falcon number"); return false;}
    
	map<int, btRigidBody *>::iterator it = sentRigidBodies.find(body_num);
    if(it == sentRigidBodies.end()){
        return false;
    }
	btRigidBody * rigidBody = it->second;

	if (falconInterfaces[falcon_num]->isGodObjectSphere())
		removeSphereGodObject(falcon_num);


	falconInterfaces[falcon_num]->setGodObject(rigidBody, minDistToMaxForce, maxDistToMaxForce, false);
	return true;

}

bool removeGodObject(int falcon_num) {
    boost::recursive_mutex::scoped_lock lock_it( collision_mutex ) ;    

    if(!check_falcon_num(falcon_num)){set_error("bad falcon number"); return false;}
    
	if (falconInterfaces[falcon_num]->isGodObjectSphere())
		removeSphereGodObject(falcon_num);


	falconInterfaces[falcon_num]->setGodObject(0, 0, 0, false);

	return true;
}


bool getCalibrated(int falcon_num){

    if(!check_falcon_num(falcon_num)){set_error("bad falcon number"); return false;}
   
#ifndef _WIN32
	
	return falcons[falcon_num]->getFalconFirmware()->isHomed();
    
#else
	hdlMakeCurrent(falconInterfaces[falcon_num]->getHDLHandle());
	return hdlGetState() == HDAL_ISREADY;
#endif
}



bool sendDynamicShape(int body_num, float * shape, int num_tris, float weight, float k, float startPos[3], float startOrient[4], float linearFactors[3], float angularFactors[3], float friction) {
    if(k<0 || k > 1){
        set_error("invalid hardness, must be between 0 and 1");
        return false;
    }
        
    boost::recursive_mutex::scoped_lock lock_it( collision_mutex ) ;    
    
    //first see if the shape number is already in there, if so, delete it first
    map<int, btRigidBody *>::iterator it = sentRigidBodies.find(body_num);
    if(it != sentRigidBodies.end()){ 
        removeDynamicShape(body_num); //alread there, delete it
    }
	
/*	btConvexHullShape * sh = new btConvexHullShape();
    for(int i=0;i<num_tris;i++){
        float * p = shape+i*3;
        btVector3 v1(p[0],p[1],p[2]);
		sh->addPoint(v1);
    }
	sh->recalcLocalAabb();
	*/
	
    btTriangleMesh * m = new btTriangleMesh();
    for(int i=0;i<num_tris;i++){
        float * p = shape+i*9;
        btVector3 v1(p[0],p[1],p[2]);
        btVector3 v2(p[3],p[4],p[5]);
        btVector3 v3(p[6],p[7],p[8]);
        m->addTriangle(v1,v2,v3);
    }
    
	btGImpactMeshShape * sh = new btGImpactMeshShape(m);
	sh->setLocalScaling(btVector3(1.f,1.f,1.f));
	sh->updateBound(); 

//    btCollisionShape * sh = new btConvexTriangleMeshShape(m,true); 

    sh->setMargin(0.0055f); 
	
    btDefaultMotionState* motionState = new btDefaultMotionState(btTransform(btQuaternion(startOrient[0],startOrient[1],startOrient[2],startOrient[3]),btVector3(startPos[0],startPos[1],startPos[2])));
	btScalar mass = weight;
    btVector3 inertia(0,0,0);

    sh->calculateLocalInertia(mass,inertia);
    btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(mass,motionState,sh,inertia);
    
	btRigidBody* rigidBody = new btRigidBody(rigidBodyCI);
	rigidBody->setRestitution(k);
	rigidBody->setLinearFactor(btVector3(linearFactors[0], linearFactors[1], linearFactors[2]));
	rigidBody->setAngularFactor(btVector3(angularFactors[0], angularFactors[1], angularFactors[2]));
	rigidBody->setFriction(friction);
//	rigidBody->setDamping(0,0);
	dynamicsWorld->addRigidBody(rigidBody);
	
	rigidBodies.push_back(rigidBody);
	sentRigidBodies[body_num] = rigidBody;

	return true;
     
}

bool removeDynamicShape(int body_num) {

    boost::recursive_mutex::scoped_lock lock_it( collision_mutex ) ;
    //delete from the collision objects array we maintain
	map<int, btRigidBody *>::iterator it = sentRigidBodies.find(body_num);
    if(it == sentRigidBodies.end()){
        return false;
    }
    for(unsigned int i = 0;i< rigidBodies.size();i++){
        if(it->second == rigidBodies[i]){
            rigidBodies.erase(rigidBodies.begin()+i);
            break;
        }
    }
    //delete from bullet
    dynamicsWorld->removeRigidBody(it->second);
    //delete actual data
	delete it->second->getMotionState();
	delete it->second->getCollisionShape();
    delete it->second;
    //delete from our list
    sentRigidBodies.erase(it);
    return true;
}

bool updateDynamicShape(int body_num, float weight, float hardness, float linearFactors[3], float angularFactors[3], float friction ) {

	boost::recursive_mutex::scoped_lock lock_it( collision_mutex ) ;

	map<int, btRigidBody *>::iterator it = sentRigidBodies.find(body_num);
    if(it == sentRigidBodies.end()){
        return false;
    }

	btRigidBody * rigidBody = it->second;
	btCollisionShape * sh = rigidBody->getCollisionShape();
	btVector3 inertia(0,0,0);
	sh->calculateLocalInertia(weight, inertia);

	rigidBody->setMassProps(weight, inertia);
	rigidBody->setRestitution(hardness);
	rigidBody->setLinearFactor(btVector3(linearFactors[0], linearFactors[1], linearFactors[2]));
	rigidBody->setAngularFactor(btVector3(angularFactors[0], angularFactors[1], angularFactors[2]));
	rigidBody->setFriction(friction);

	return true;
}

bool setDynamicShapePose(int body_num,float startPos[3], float startOrient[4]) {
	boost::recursive_mutex::scoped_lock lock_it( collision_mutex ) ;

	map<int, btRigidBody *>::iterator it = sentRigidBodies.find(body_num);
    if(it == sentRigidBodies.end()){
        return false;
    }
	btRigidBody * rigidBody = it->second;

	rigidBody->setWorldTransform(btTransform(btQuaternion(startOrient[0],startOrient[1],startOrient[2],startOrient[3]),btVector3(startPos[0],startPos[1],startPos[2])));
	rigidBody->setLinearVelocity(btVector3(0,0,0));
	rigidBody->setAngularVelocity(btVector3(0,0,0));

	return true;
}

bool applyForceToShape(int body_num, float linear[3], float torque[3]) {
	boost::recursive_mutex::scoped_lock lock_it( collision_mutex ) ;

	map<int, btRigidBody *>::iterator it = sentRigidBodies.find(body_num);
    if(it == sentRigidBodies.end()){
        return false;
    }
	btRigidBody * rigidBody = it->second;

	rigidBody->applyCentralForce(btVector3(linear[0],linear[1],linear[2]));
	rigidBody->applyTorque(btVector3(torque[0],torque[1],torque[2]));

	rigidBody->activate();

	return true;
}

bool getDynamicShapePose(int body_num, float pos[3], float orient[4]) {

	boost::recursive_mutex::scoped_lock lock_it( collision_mutex ) ;

	map<int, btRigidBody *>::iterator it = sentRigidBodies.find(body_num);
    if(it == sentRigidBodies.end()){
        return false;
    }
	btTransform trans;
	it->second->getMotionState()->getWorldTransform(trans);
	btVector3 p = trans.getOrigin();
	btQuaternion q = trans.getRotation();

	pos[0] = p.getX();
	pos[1] = p.getY();
	pos[2] = p.getZ();
	
	orient[0] = q.getX();
	orient[1] = q.getY();
	orient[2] = q.getZ();
	orient[3] = q.getW();

	return true;
}


bool addSpringToShape(int body_num, int spring_num, float max_force, float dampingFactor, float goalPos[3], float goalOrient[4], float posConstraintLower[3], float posConstraintUpper[3], float orientConstraintLower[3], float orientConstraintUpper[3], int directionality[6]) {
	boost::recursive_mutex::scoped_lock lock_it( collision_mutex ) ;

	map<int, btRigidBody *>::iterator it = sentRigidBodies.find(body_num);
    if(it == sentRigidBodies.end()){
        return false;
    }
	btRigidBody * b = it->second;

	btVector3 goal(goalPos[0], goalPos[1], goalPos[2]);
	btVector3 pConstl(posConstraintLower[0], posConstraintLower[1], posConstraintLower[2]);
	btVector3 pConstu(posConstraintUpper[0], posConstraintUpper[1], posConstraintUpper[2]);
	btQuaternion orient(goalOrient[0], goalOrient[1], goalOrient[2], goalOrient[3]);
	btVector3 oConstl(orientConstraintLower[0], orientConstraintLower[1], orientConstraintLower[2]);
	btVector3 oConstu(orientConstraintUpper[0], orientConstraintUpper[1], orientConstraintUpper[2]);

	DampedSpring * so = new DampedSpring(b, max_force, dampingFactor, goal, orient, pConstl, pConstu, oConstl, oConstu, &(directionality[0]));

	springs[spring_num] = so;

	return true;
}

bool removeSpring(int spring_num) {
	boost::recursive_mutex::scoped_lock lock_it( collision_mutex ) ;

	map<int, DampedSpring *>::iterator it = springs.find(spring_num);
    if(it == springs.end()){
        return false;
    }

	delete it->second;
	springs.erase(it);

	return true;
}

bool setSpring(int spring_num, float max_force, float dampingFactor, float goalPos[3], float goalOrient[4], float posConstraintLower[3], float posConstraintUpper[3], float orientConstraintLower[3], float orientConstraintUpper[3], int directionality[6]) {
	boost::recursive_mutex::scoped_lock lock_it( collision_mutex ) ;

	map<int, DampedSpring *>::iterator it = springs.find(spring_num);
    if(it == springs.end()){
        return false;
    }

	btVector3 goal(goalPos[0], goalPos[1], goalPos[2]);
	btVector3 pConstl(posConstraintLower[0], posConstraintLower[1], posConstraintLower[2]);
	btVector3 pConstu(posConstraintUpper[0], posConstraintUpper[1], posConstraintUpper[2]);
	btQuaternion orient(goalOrient[0], goalOrient[1], goalOrient[2], goalOrient[3]);
	btVector3 oConstl(orientConstraintLower[0], orientConstraintLower[1], orientConstraintLower[2]);
	btVector3 oConstu(orientConstraintUpper[0], orientConstraintUpper[1], orientConstraintUpper[2]);

	it->second->setParams(max_force, dampingFactor, goal, orient, pConstl, pConstu, oConstl, oConstu, &(directionality[0]));

	return true;
}


bool lerpSpring(int spring_num, float max_force, float dampingFactor, float goalPos[3], float goalOrient[4], float posConstraintLower[3], float posConstraintUpper[3], float orientConstraintLower[3], float orientConstraintUpper[3], int directionality[6], float time_in_secs) {
	boost::recursive_mutex::scoped_lock lock_it( collision_mutex ) ;

	map<int, DampedSpring *>::iterator it = springs.find(spring_num);
    if(it == springs.end()){
        return false;
    }

	btVector3 goal(goalPos[0], goalPos[1], goalPos[2]);
	btVector3 pConstl(posConstraintLower[0], posConstraintLower[1], posConstraintLower[2]);
	btVector3 pConstu(posConstraintUpper[0], posConstraintUpper[1], posConstraintUpper[2]);
	btQuaternion orient(goalOrient[0], goalOrient[1], goalOrient[2], goalOrient[3]);
	btVector3 oConstl(orientConstraintLower[0], orientConstraintLower[1], orientConstraintLower[2]);
	btVector3 oConstu(orientConstraintUpper[0], orientConstraintUpper[1], orientConstraintUpper[2]);

	it->second->lerpParams(max_force, dampingFactor, goal, orient, pConstl, pConstu, oConstl, oConstu, &(directionality[0]), time_in_secs);

	return true;
}

float getFPS() {
	boost::recursive_mutex::scoped_lock lock_it( collision_mutex ) ;

	float sum = 0;

	for (int i = 0; i < TOTAL_FRAMES; i ++) {
		sum += frame_times[i];
	}

	return 1.0f / ( sum / TOTAL_FRAMES);
}


bool startLog(int falcon_num) {
//	falconInterfaces[falcon_num]->startLogging();
	return false;
}

bool stopLog(int falcon_num, int & num_entries, float * god_obj_positions, float * haptic_tip_positions, float * god_obj_forces, float * haptic_tip_forces, float * delta_ts) {

//	falconInterfaces[falcon_num]->stopLogging();
	return false;
}


bool setGravity(float force[3]) {
	boost::recursive_mutex::scoped_lock lock_it( collision_mutex ) ;

	dynamicsWorld->setGravity(btVector3(force[0],force[1],force[2]));
	return true;
}


bool updateHapticTransform(int falcon_num, float * pos, float * rot, float * scale, bool useCompensator, float time_in_secs) {

   boost::recursive_mutex::scoped_lock lock_it( collision_mutex ) ;    

    if(!check_falcon_num(falcon_num)){set_error("bad falcon number"); return false;}

	falconInterfaces[falcon_num]->setTransform(btTransform(btQuaternion(rot[0], rot[1], rot[2], rot[3]), btVector3(pos[0], pos[1], pos[2])), btVector3(scale[0], scale[1], scale[2]), useCompensator, time_in_secs);

	return true;
}


bool falcon_update_all(std::vector<_falconunity_haptic_tip_params *> haptic_params, std::vector<_falconunity_spring_params *> spring_params, float * fps, std::vector<_falconunity_falcon_params *> * falcon_params, std::vector<_falconunity_object_params *> * object_params) {

	boost::recursive_mutex::scoped_lock lock_it( collision_mutex ) ;

	for (unsigned int i = 0; i < haptic_params.size(); i ++) {
		updateHapticTransform(haptic_params[i]->falcon_num, haptic_params[i]->pos, haptic_params[i]->orient, haptic_params[i]->scale, haptic_params[i]->useCompensator, haptic_params[i]->time_in_secs);
	}

	for (unsigned int i = 0; i < spring_params.size(); i ++) {
		lerpSpring(spring_params[i]->spring_num, spring_params[i]->max_force, spring_params[i]->dampingFactor, spring_params[i]->goalPos, spring_params[i]->goalOrient, spring_params[i]->posConstraintLower, spring_params[i]->posConstraintUpper, spring_params[i]->orientConstraintLower, spring_params[i]->orientConstraintUpper, spring_params[i]->directionality, spring_params[i]->time_in_secs);
	}

	for (unsigned int i = 0; i < falconInterfaces.size(); i ++) {
		_falconunity_falcon_params * temp = new _falconunity_falcon_params();
		
		getFalconButtonStates(i, temp->buttons);

		getGodPosition(i, temp->godPositions);
		getTipPosition(i, temp->tipPositions);
		getFalconForces(i, temp->curforces);
		temp->falcon_num = i;

		(*falcon_params).push_back(temp);
	}

	for (std::map<int,btRigidBody *>::iterator it=sentRigidBodies.begin(); it!=sentRigidBodies.end(); ++it) {
		_falconunity_object_params * temp = new _falconunity_object_params();
		
		temp->object_num = it->first;
		getDynamicShapePose(it->first, temp->pos, temp->orient);
		
		(*object_params).push_back(temp);
	}

	*fps = getFPS();

	return true;
}


BOOL WINAPI DllMain(
    HINSTANCE hinstDLL,  // handle to DLL module
    DWORD fdwReason,     // reason for calling function
    LPVOID lpReserved )  // reserved
{
    // Perform actions based on the reason for calling.
    switch( fdwReason ) 
    { 
        case DLL_PROCESS_ATTACH:
         // Initialize once for each new process.
         // Return FALSE to fail DLL load.
			done = true;
            break;

        case DLL_THREAD_ATTACH:
         // Do thread-specific initialization.
            break;

        case DLL_THREAD_DETACH:
         // Do thread-specific cleanup.
            break;

        case DLL_PROCESS_DETACH:
         // Perform any necessary cleanup.

			closeFalconUnity();
            break;
    }
    return TRUE;  // Successful DLL_PROCESS_ATTACH.
}
