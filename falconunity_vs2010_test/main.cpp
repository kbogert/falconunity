//  falconunity_test
//
//  Scratch program for testing the falconunity dll directly
//
//
//  Created by Kyle Johnsen on 4/29/12.
//  Copyright (c) 2012 The University of Georgia.
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


#include <iostream>
using namespace std;
#include "falconunity/falconunity.h"
int main(int argc, char ** argv){
    
   // initFalconUnity();
    float verts[] = {-20, 0, 20,
                     -20, 0, -20,
                    20, 0, 20,
                    20, 0, 20,
                    -20,0,-20,
        20,0,-20};
       float force[3] = {0,0,0};
 //   setForceField(0, force);
 //   sendWorldShape(0,verts, 2);
 //   sendWorldShape(0,verts, 2);
    
	   float startPos[3] = {0,-0.025,0};
	   float startOri[4] = {0,0,0,1};

    initFalconUnity();
	float startP[3] = {0,0,0};
	float scale[3] = {1,1,-1};

	setSphereGodObject(0, 0.005, .1, startP, 0.05f, 0.15f);

	//	calibrate(0, 25, 0);
//	while(true) {};
	float lf[3] = {0,1,0};
	float af[3] = {0,0,0};
    sendDynamicShape(0,verts, 2, 1, 1, startPos, startOri, lf, af, 1);
	float maxPos[3] = {0,0,0};
	float minPos[3] = {0,-.005,0};
	float maxOri[3] = {0,0,0};
	float minOri[3] = {0,0,0};
	int dir[6] = {0,0,0,0,0,0};

	addSpringToShape(0, 0,1, 1, startPos, startOri, minPos, maxPos, minOri, maxOri, dir);

//    sendWorldShape(0,verts, 2, 4000);
    setForceField(0, force);
//	 setTipPositionScale(0, scale);
//	startSim();
    int i=0;
	float forces[3];
    while(true){
        //printf("%d ",i++);
//		stepSim();
        float pos[3];
        float ori[4];


		int buttons[4] = {0,0,0,0};
       
			   
		getTipPosition(0, pos);
 //       cout << "tip = " << pos[0] << "," << pos[1] << "," << pos[2] << endl;
		getGodPosition(0, pos);
//        cout << "god = " << pos[0] << "," << pos[1] << "," << pos[2] << endl;
		getDynamicShapePose(0,pos,ori);
//        cout << "position = " << pos[0] << "," << pos[1] << "," << pos[2] << " ori = " << ori[0] <<","<<ori[1]<<","<<ori[2]<<","<<ori[3]<<endl;
		lerpSpring(0,1, 1, startPos, startOri, minPos, maxPos, minOri, maxOri,dir,1);
		updateHapticTransform(0,maxPos, startOri,lf, true, 1/60.0f);
		getFalconForces(0,forces);
		cout << "FPS: " << getFPS() << " Forces: " << forces[0] << ", "<< forces[1] << ", "<< forces[2]  << endl;;
		//getFalconButtonStates(0,buttons);
        //cout << "buttons = ";
        /*
        for(int i=0;i<4;i++){
            cout << buttons[i] << ",";
        }
        cout << endl;
        */
    }
    
    return 0;
}