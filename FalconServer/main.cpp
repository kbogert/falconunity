//  FalconServer
//
//  A small server program that manages the FalconUnity physics world
//  for a client.  Created so that non-pro versions of unity can use
//  FalconUnity.
//
/*  Created by Kenneth Bogert
    Copyright (c) 2013 The University of Georgia.

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
#undef UNICODE

#define WIN32_LEAN_AND_MEAN

#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>

// Need to link with Ws2_32.lib
#pragma comment (lib, "Ws2_32.lib")
// #pragma comment (lib, "Mswsock.lib")

#define DEFAULT_BUFLEN 1024
#define DEFAULT_PORT "8747"
#define PROTOCOL_VERSION_NUM 1


#include <iostream>
#include <cstring>
#include <string>
#include <vector>
#include <sstream>
using namespace std;


#include "falconunity.h"

bool hasStarted;
string packetbuf;

// stolen from: http://stackoverflow.com/questions/236129/splitting-a-string-in-c
std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}


std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

// iteratively builds and returns a packet minus the footer 
// returns 1 if a full packet has been received and fullPacket is set
// returns 0 if a full packet hasn't been received yet
// returns -1 if reading from the socket has an error
int recv_packet(SOCKET & socket, string ** fullPacket) {
	char recvbuf[DEFAULT_BUFLEN];

	int iResult = recv(socket, recvbuf, DEFAULT_BUFLEN, 0);

	if (iResult <= 0) {
		// no data received or error, close connection
		packetbuf.clear();

		if (iResult < 0) {
			 printf("recv failed with error: %ld\n", WSAGetLastError());
		}

		return -1;
	}

	packetbuf += string(recvbuf, iResult);


	int location = packetbuf.find("\n\n");

	if (location >= 0) {
		// we've found a packet! extract from packetbuf and return to the user

		string * returnval = new string (packetbuf.substr(0,location + 1));

		packetbuf.clear();

		*fullPacket = returnval;

		return 1;
	}

	return 0;
}

bool send_all(SOCKET & socket, const char * data, int * len) {
//	printf("Sending: %s\n", data);
    int total = 0;        // how many bytes we've sent
    int bytesleft = *len; // how many we have left to send
    int n;

    while(total < *len) {
        n = send(socket, data+total, bytesleft, 0);
        if (n == -1) { break; }
        total += n;
        bytesleft -= n;
    }

    *len = total; // return number actually sent here

    return n!=-1; // return false on failure, true on success
}

void print_packet_error(char * msg, string * packet) {
	printf("Error: %s processing packet: %s\n", msg, packet->c_str());
}

// interpret and parse the packet here, then make the appropriate call into falconunity, then convert results into a packet and sendall
bool handle_packet(string * packet, SOCKET & socket) {

	// the first line is the name of the command, extract it and convert to a command num

	int pos = packet->find("\n");

	if (pos < 0) {
		print_packet_error("Command not found", packet);
		return false;
	}

			
	// break the packet into parameters
	vector<string> lines = split(*packet, '\n');

	bool returnval = false;

	if (lines[0].compare("VERSION") == 0) {
		stringstream out;
		out << PROTOCOL_VERSION_NUM << "\n\n";

		string s = out.str();
		int size = s.size();
		return send_all(socket, s.c_str(), &size);
	}

	if (lines[0].compare("START") == 0) {
		if (hasStarted) {

			closeFalconUnity();
			cout << "Physics world stopped" << endl;
		}

		int num_falcons = initFalconUnity();

		hasStarted = true;

		cout << "Physics world started with " << num_falcons << " Falcon(s)" << endl;

		stringstream out;
		out << "TRUE" << "\n";
		out << num_falcons << "\n\n";

		string s = out.str();
		int size = s.size();
		return send_all(socket, s.c_str(), &size);

	} else if (lines[0].compare("STOP") == 0) {

		if (hasStarted)
			closeFalconUnity();

		hasStarted = false;

		cout << "Physics world stopped" << endl;

		string s = "TRUE\n\n";
		int size = s.size();
		return send_all(socket, s.c_str(), &size);
	}

	if (!hasStarted) {
		initFalconUnity();

		hasStarted = true;
	}

	if (lines[0].compare("UPDATE") == 0) {
		// params are:  H, falcon_num, pos, orient, scale, useCompensator, time_in_secs 
		//				S, spring_num, max_force, goalPos, goalOrient, posConstraintLower, posConstraintUpper, orientConstraintLower, orientConstraintUpper, time_in_secs
		// there may be any number of these

		vector<_falconunity_spring_params *> spring_params;
		vector<_falconunity_haptic_tip_params *> haptic_params;

		unsigned int i = 1;


		while ( i < lines.size() ) {

			if (lines[i].compare("H") == 0) {
				// haptic tip update
				if (lines.size() - i < 7) {
					print_packet_error("Incorrect number of parameters for command", packet);
					return false;
				}

				_falconunity_haptic_tip_params * params = new _falconunity_haptic_tip_params();
				sscanf_s(lines[i+1].c_str(), "%d", & params->falcon_num);

				vector<string> elems = split(lines[i+2], '\t');
				if (elems.size() != 3) {
					print_packet_error("Error in parameter 2", packet);
					return false;
				}
				sscanf_s(elems[0].c_str(), "%G", & params->pos[0]);
				sscanf_s(elems[1].c_str(), "%G", & params->pos[1]);
				sscanf_s(elems[2].c_str(), "%G", & params->pos[2]);

				elems = split(lines[i+3], '\t');
				if (elems.size() != 4) {
					print_packet_error("Error in parameter 3", packet);
					return false;
				}
				sscanf_s(elems[0].c_str(), "%G", & params->orient[0]);
				sscanf_s(elems[1].c_str(), "%G", & params->orient[1]);
				sscanf_s(elems[2].c_str(), "%G", & params->orient[2]);
				sscanf_s(elems[3].c_str(), "%G", & params->orient[3]);

				elems = split(lines[i+4], '\t');
				if (elems.size() != 3) {
					print_packet_error("Error in parameter 4", packet);
					return false;
				}
				sscanf_s(elems[0].c_str(), "%G", & params->scale[0]);
				sscanf_s(elems[1].c_str(), "%G", & params->scale[1]);
				sscanf_s(elems[2].c_str(), "%G", & params->scale[2]);

				int c;
				sscanf_s(lines[i+5].c_str(), "%d", & c);
				params->useCompensator = (c != 0);
				sscanf_s(lines[i+6].c_str(), "%G", & params->time_in_secs);

				haptic_params.push_back(params);

				i += 7;

			} else {
				// spring update

				if (lines.size() - i < 12) {
					print_packet_error("Incorrect number of parameters for command", packet);
					return false;
				}

			
				_falconunity_spring_params * params = new _falconunity_spring_params();

				sscanf_s(lines[i+1].c_str(), "%d", & params->spring_num);
				sscanf_s(lines[i+2].c_str(), "%G", & params->max_force);
				sscanf_s(lines[i+3].c_str(), "%G", & params->dampingFactor);
			
				vector<string> elems = split(lines[i+4], '\t');
				if (elems.size() != 3) {
					print_packet_error("Error in parameter 3", packet);
					return false;
				}
				sscanf_s(elems[0].c_str(), "%G", & params->goalPos[0]);
				sscanf_s(elems[1].c_str(), "%G", & params->goalPos[1]);
				sscanf_s(elems[2].c_str(), "%G", & params->goalPos[2]);

				elems = split(lines[i+5], '\t');
				if (elems.size() != 4) {
					print_packet_error("Error in parameter 4", packet);
					return false;
				}
				sscanf_s(elems[0].c_str(), "%G", & params->goalOrient[0]);
				sscanf_s(elems[1].c_str(), "%G", & params->goalOrient[1]);
				sscanf_s(elems[2].c_str(), "%G", & params->goalOrient[2]);
				sscanf_s(elems[3].c_str(), "%G", & params->goalOrient[3]);

				elems = split(lines[i+6], '\t');
				if (elems.size() != 3) {
					print_packet_error("Error in parameter 5", packet);
					return false;
				}
				sscanf_s(elems[0].c_str(), "%G", & params->posConstraintLower[0]);
				sscanf_s(elems[1].c_str(), "%G", & params->posConstraintLower[1]);
				sscanf_s(elems[2].c_str(), "%G", & params->posConstraintLower[2]);

				elems = split(lines[i+7], '\t');
				if (elems.size() != 3) {
					print_packet_error("Error in parameter 6", packet);
					return false;
				}
				sscanf_s(elems[0].c_str(), "%G", & params->posConstraintUpper[0]);
				sscanf_s(elems[1].c_str(), "%G", & params->posConstraintUpper[1]);
				sscanf_s(elems[2].c_str(), "%G", & params->posConstraintUpper[2]);

				elems = split(lines[i+8], '\t');
				if (elems.size() != 3) {
					print_packet_error("Error in parameter 7", packet);
					return false;
				}
				sscanf_s(elems[0].c_str(), "%G", & params->orientConstraintLower[0]);
				sscanf_s(elems[1].c_str(), "%G", & params->orientConstraintLower[1]);
				sscanf_s(elems[2].c_str(), "%G", & params->orientConstraintLower[2]);

				elems = split(lines[i+9], '\t');
				if (elems.size() != 3) {
					print_packet_error("Error in parameter 8", packet);
					return false;
				}
				sscanf_s(elems[0].c_str(), "%G", & params->orientConstraintUpper[0]);
				sscanf_s(elems[1].c_str(), "%G", & params->orientConstraintUpper[1]);
				sscanf_s(elems[2].c_str(), "%G", & params->orientConstraintUpper[2]);

				elems = split(lines[i+10], '\t');
				if (elems.size() != 6) {
					print_packet_error("Error in parameter 9", packet);
					return false;
				}
				sscanf_s(elems[0].c_str(), "%d", & params->directionality[0]);
				sscanf_s(elems[1].c_str(), "%d", & params->directionality[1]);
				sscanf_s(elems[2].c_str(), "%d", & params->directionality[2]);
				sscanf_s(elems[3].c_str(), "%d", & params->directionality[3]);
				sscanf_s(elems[4].c_str(), "%d", & params->directionality[4]);
				sscanf_s(elems[5].c_str(), "%d", & params->directionality[5]);


				sscanf_s(lines[i+11].c_str(), "%G", & params->time_in_secs);

				spring_params.push_back(params);

				i += 12;
			}
		}

		// get read to recieve update data from falconunity
		float fps;
		vector <_falconunity_falcon_params *> falcon_params;
		vector <_falconunity_object_params *> object_params;

		returnval = falcon_update_all(haptic_params, spring_params, &fps, &falcon_params, &object_params);

		for (unsigned int i = 0; i < spring_params.size(); i ++) {
			delete spring_params[i];
		}

		for (unsigned int i = 0; i < haptic_params.size(); i ++) {
			delete haptic_params[i];
		}

		// now build the response packet and send it
		stringstream out;
		out << (returnval ? "TRUE" : "FALSE") << "\n";
		out << fps << "\n";
		out << falcon_params.size() << "\n";

		for (unsigned int i = 0; i < falcon_params.size(); i ++) {
			out << falcon_params[i]->tipPositions[0] << '\t' << falcon_params[i]->tipPositions[1] << '\t' << falcon_params[i]->tipPositions[2] << '\n';
			out << falcon_params[i]->godPositions[0] << '\t' << falcon_params[i]->godPositions[1] << '\t' << falcon_params[i]->godPositions[2] << '\n';
			out << falcon_params[i]->curforces[0] << '\t' << falcon_params[i]->curforces[1] << '\t' << falcon_params[i]->curforces[2] << '\n';
			out << falcon_params[i]->buttons[0] << '\t' << falcon_params[i]->buttons[1] << '\t' << falcon_params[i]->buttons[2] << '\t' << falcon_params[i]->buttons[3] << '\n';
			delete falcon_params[i];
		}

		out << object_params.size() << "\n";
		for (unsigned int i = 0; i < object_params.size(); i ++) {
			out << object_params[i]->object_num << '\t' << object_params[i]->pos[0] << '\t' << object_params[i]->pos[1] << '\t' << object_params[i]->pos[2] << '\t'
				<< object_params[i]->orient[0] << '\t' << object_params[i]->orient[1] << '\t' << object_params[i]->orient[2] << '\t' << object_params[i]->orient[3] << '\t' << object_params[i]->orient[4] << '\n';
			delete object_params[i];
		}

		out << "\n";

		string s = out.str();
		int size = s.size();
		returnval = send_all(socket, s.c_str(), &size);

	} else if (lines[0].compare("KEEPALIVE") == 0) {
		// nice to know
		
		
		string s = "TRUE\n\n";
		int size = s.size();
		returnval = send_all(socket, s.c_str(), &size);

	} else if (lines[0].compare("GETERROR") == 0) {

		char buf[1024];

		getLastError(buf);
		string s = buf;
		s += "\n\n";
		int size = s.size();
		returnval = send_all(socket, s.c_str(), &size);

	} else if (lines[0].compare("SETFORCEFIELD") == 0) {
		// params = which falcon, forces to send in each direction
		
		if (lines.size() < 3) {
			print_packet_error("Incorrect number of parameters for command", packet);
			return false;
		}

		int falcon_num;
		sscanf_s(lines[1].c_str(), "%d", & falcon_num);

		vector<string> elems = split(lines[2], '\t');
		if (elems.size() != 3) {
			print_packet_error("Error in parameter 2", packet);
			return false;
		}

		float forces[3];
		
		sscanf_s(elems[0].c_str(), "%G", & forces[0]);
		sscanf_s(elems[1].c_str(), "%G", & forces[1]);
		sscanf_s(elems[2].c_str(), "%G", & forces[2]);

		returnval = setForceField(falcon_num, forces);

		stringstream out;
		out << (returnval ? "TRUE" : "FALSE") << "\n\n";
		string s = out.str();
		int size = s.size();
		returnval = send_all(socket, s.c_str(), &size);

	} else if (lines[0].compare("SETSPHEREGODOBJECT") == 0) {
		// params: falcon_num, radius, mass, pos[3], minDistToMaxForce, maxDistToMaxForce

		if (lines.size() < 7) {
			print_packet_error("Incorrect number of parameters for command", packet);
			return false;
		}

		int falcon_num;
		float radius;
		float mass;
		float pos[3];
		float minDistToMaxForce;
		float maxDistToMaxForce;
		
		sscanf_s(lines[1].c_str(), "%d", & falcon_num);
		sscanf_s(lines[2].c_str(), "%G", & radius);
		sscanf_s(lines[3].c_str(), "%G", & mass);
		sscanf_s(lines[5].c_str(), "%G", & minDistToMaxForce);
		sscanf_s(lines[6].c_str(), "%G", & maxDistToMaxForce);

		vector<string> elems = split(lines[4], '\t');
		if (elems.size() != 3) {
			print_packet_error("Error in parameter 4", packet);
			return false;
		}

		sscanf_s(elems[0].c_str(), "%G", & pos[0]);
		sscanf_s(elems[1].c_str(), "%G", & pos[1]);
		sscanf_s(elems[2].c_str(), "%G", & pos[2]);


		returnval = setSphereGodObject(falcon_num, radius, mass, pos, minDistToMaxForce, maxDistToMaxForce);


		stringstream out;
		out << (returnval ? "TRUE" : "FALSE") << "\n\n";
		string s = out.str();
		int size = s.size();
		returnval = send_all(socket, s.c_str(), &size);

	} else if (lines[0].compare("SETRIGIDBODYGODOBJECT") == 0) {
		// params: falcon_num, body_num, minDistToMaxForce, maxDistToMaxForce

		if (lines.size() < 5) {
			print_packet_error("Incorrect number of parameters for command", packet);
			return false;
		}

		int falcon_num;
		int body_num;
		float minDistToMaxForce;
		float maxDistToMaxForce;
		
		sscanf_s(lines[1].c_str(), "%d", & falcon_num);
		sscanf_s(lines[2].c_str(), "%d", & body_num);
		sscanf_s(lines[3].c_str(), "%G", & minDistToMaxForce);
		sscanf_s(lines[4].c_str(), "%G", & maxDistToMaxForce);

		returnval = setRigidBodyGodObject(falcon_num, body_num, minDistToMaxForce, maxDistToMaxForce);


		stringstream out;
		out << (returnval ? "TRUE" : "FALSE") << "\n\n";
		string s = out.str();
		int size = s.size();
		returnval = send_all(socket, s.c_str(), &size);

	} else if (lines[0].compare("REMOVEGODOBJECT") == 0) {
		// params: falcon_num

		if (lines.size() < 2) {
			print_packet_error("Incorrect number of parameters for command", packet);
			return false;
		}

		int falcon_num;
		
		sscanf_s(lines[1].c_str(), "%d", & falcon_num);


		returnval = removeGodObject(falcon_num);


		stringstream out;
		out << (returnval ? "TRUE" : "FALSE") << "\n\n";
		string s = out.str();
		int size = s.size();
		returnval = send_all(socket, s.c_str(), &size);

	} else if (lines[0].compare("SENDDYNAMICSHAPE") == 0) {
		// params: body_num, shape, num_tris, weight, hardness, startPos[3], startOrient[4], linearFactors[3], angularFactors[3], friction

		if (lines.size() < 11) {
			print_packet_error("Incorrect number of parameters for command", packet);
			return false;
		}

		int body_num;
		int num_tris;
		float * tris;
		float weight;
		float hardness;
		float startPos[3];
		float startOrient[4];
		float linearFactors[3];
		float angularFactors[3];
		float friction;

		sscanf_s(lines[1].c_str(), "%d", & body_num);
		sscanf_s(lines[3].c_str(), "%d", & num_tris);
		sscanf_s(lines[4].c_str(), "%G", & weight);
		sscanf_s(lines[5].c_str(), "%G", & hardness);
		sscanf_s(lines[10].c_str(), "%G", & friction);

		vector<string> elems = split(lines[2], '\t');
		tris = new float[elems.size()];
		for (unsigned int i = 0; i < elems.size(); i ++) {
			sscanf_s(elems[i].c_str(), "%G", & tris[i]);
		}

		elems = split(lines[6], '\t');
		if (elems.size() != 3) {
			print_packet_error("Error in parameter 6", packet);
			return false;
		}

		sscanf_s(elems[0].c_str(), "%G", & startPos[0]);
		sscanf_s(elems[1].c_str(), "%G", & startPos[1]);
		sscanf_s(elems[2].c_str(), "%G", & startPos[2]);


		elems = split(lines[7], '\t');
		if (elems.size() != 4) {
			print_packet_error("Error in parameter 7", packet);
			return false;
		}

		sscanf_s(elems[0].c_str(), "%G", & startOrient[0]);
		sscanf_s(elems[1].c_str(), "%G", & startOrient[1]);
		sscanf_s(elems[2].c_str(), "%G", & startOrient[2]);
		sscanf_s(elems[3].c_str(), "%G", & startOrient[3]);

		elems = split(lines[8], '\t');
		if (elems.size() != 3) {
			print_packet_error("Error in parameter 8", packet);
			return false;
		}

		sscanf_s(elems[0].c_str(), "%G", & linearFactors[0]);
		sscanf_s(elems[1].c_str(), "%G", & linearFactors[1]);
		sscanf_s(elems[2].c_str(), "%G", & linearFactors[2]);

		elems = split(lines[9], '\t');
		if (elems.size() != 3) {
			print_packet_error("Error in parameter 9", packet);
			return false;
		}

		sscanf_s(elems[0].c_str(), "%G", & angularFactors[0]);
		sscanf_s(elems[1].c_str(), "%G", & angularFactors[1]);
		sscanf_s(elems[2].c_str(), "%G", & angularFactors[2]);

		returnval = sendDynamicShape(body_num, tris, num_tris, weight, hardness, startPos, startOrient, linearFactors, angularFactors, friction);

		delete tris;

		stringstream out;
		out << (returnval ? "TRUE" : "FALSE") << "\n\n";
		string s = out.str();
		int size = s.size();
		returnval = send_all(socket, s.c_str(), &size);

	} else if (lines[0].compare("REMOVEDYNAMICSHAPE") == 0) {
		// params: body_num

		if (lines.size() < 2) {
			print_packet_error("Incorrect number of parameters for command", packet);
			return false;
		}

		int body_num;

		sscanf_s(lines[1].c_str(), "%d", & body_num);

		returnval = removeDynamicShape(body_num);

		stringstream out;
		out << (returnval ? "TRUE" : "FALSE") << "\n\n";
		string s = out.str();
		int size = s.size();
		returnval = send_all(socket, s.c_str(), &size);

	} else if (lines[0].compare("UPDATEDYNAMICSHAPE") == 0) {
		// params: body_num, weight, hardness, linearFactors[3], angularFactors[3], friction 

		if (lines.size() < 7) {
			print_packet_error("Incorrect number of parameters for command", packet);
			return false;
		}

		int body_num;
		float weight;
		float hardness;
		float linearFactors[3];
		float angularFactors[3];
		float friction;

		sscanf_s(lines[1].c_str(), "%d", & body_num);
		sscanf_s(lines[2].c_str(), "%G", & weight);
		sscanf_s(lines[3].c_str(), "%G", & hardness);
		sscanf_s(lines[6].c_str(), "%G", & friction);

		vector<string> elems = split(lines[4], '\t');
		if (elems.size() != 3) {
			print_packet_error("Error in parameter 4", packet);
			return false;
		}

		sscanf_s(elems[0].c_str(), "%G", & linearFactors[0]);
		sscanf_s(elems[1].c_str(), "%G", & linearFactors[1]);
		sscanf_s(elems[2].c_str(), "%G", & linearFactors[2]);

		elems = split(lines[5], '\t');
		if (elems.size() != 3) {
			print_packet_error("Error in parameter 5", packet);
			return false;
		}

		sscanf_s(elems[0].c_str(), "%G", & angularFactors[0]);
		sscanf_s(elems[1].c_str(), "%G", & angularFactors[1]);
		sscanf_s(elems[2].c_str(), "%G", & angularFactors[2]);

		returnval = updateDynamicShape(body_num,  weight, hardness, linearFactors, angularFactors, friction);

		stringstream out;
		out << (returnval ? "TRUE" : "FALSE") << "\n\n";
		string s = out.str();
		int size = s.size();
		returnval = send_all(socket, s.c_str(), &size);

	} else if (lines[0].compare("SETDYNAMICSHAPE") == 0) {
		// params: body_num, pos[3], orient[4]

		if (lines.size() < 4) {
			print_packet_error("Incorrect number of parameters for command", packet);
			return false;
		}

		int body_num;
		float pos[3];
		float orient[4];

		sscanf_s(lines[1].c_str(), "%d", & body_num);

		vector<string> elems = split(lines[2], '\t');
		if (elems.size() != 3) {
			print_packet_error("Error in parameter 2", packet);
			return false;
		}

		sscanf_s(elems[0].c_str(), "%G", & pos[0]);
		sscanf_s(elems[1].c_str(), "%G", & pos[1]);
		sscanf_s(elems[2].c_str(), "%G", & pos[2]);

		elems = split(lines[3], '\t');
		if (elems.size() != 4) {
			print_packet_error("Error in parameter 3", packet);
			return false;
		}

		sscanf_s(elems[0].c_str(), "%G", & orient[0]);
		sscanf_s(elems[1].c_str(), "%G", & orient[1]);
		sscanf_s(elems[2].c_str(), "%G", & orient[2]);
		sscanf_s(elems[3].c_str(), "%G", & orient[3]);

		returnval = setDynamicShapePose(body_num, pos, orient);

		stringstream out;
		out << (returnval ? "TRUE" : "FALSE") << "\n\n";
		string s = out.str();
		int size = s.size();
		returnval = send_all(socket, s.c_str(), &size);


	} else if (lines[0].compare("ADDSPRINGTOSHAPE") == 0) {
		// params: body_num, spring_num, max_force, dampingFactor, goalPos[3], goalOrient[4], posConstraintLower[3], posConstraintUpper[3], orientConstraintLower[3], orientConstraintUpper[3], directionality[6]

		if (lines.size() < 12) {
			print_packet_error("Incorrect number of parameters for command", packet);
			return false;
		}

		int body_num;
		int spring_num;
		float max_force;
		float dampingFactor;
		float goalPos[3];
		float goalOrient[4];
		float pConstL[3];
		float pConstU[3];
		float oConstL[3];
		float oConstU[3];
		int directionality[6];

		sscanf_s(lines[1].c_str(), "%d", & body_num);
		sscanf_s(lines[2].c_str(), "%d", & spring_num);
		sscanf_s(lines[3].c_str(), "%G", & max_force);
		sscanf_s(lines[4].c_str(), "%G", & dampingFactor);


		vector<string> elems = split(lines[5], '\t');
		if (elems.size() != 3) {
			print_packet_error("Error in parameter 4", packet);
			return false;
		}

		sscanf_s(elems[0].c_str(), "%G", & goalPos[0]);
		sscanf_s(elems[1].c_str(), "%G", & goalPos[1]);
		sscanf_s(elems[2].c_str(), "%G", & goalPos[2]);


		elems = split(lines[6], '\t');
		if (elems.size() != 4) {
			print_packet_error("Error in parameter 5", packet);
			return false;
		}

		sscanf_s(elems[0].c_str(), "%G", & goalOrient[0]);
		sscanf_s(elems[1].c_str(), "%G", & goalOrient[1]);
		sscanf_s(elems[2].c_str(), "%G", & goalOrient[2]);
		sscanf_s(elems[3].c_str(), "%G", & goalOrient[3]);

		elems = split(lines[7], '\t');
		if (elems.size() != 3) {
			print_packet_error("Error in parameter 6", packet);
			return false;
		}

		sscanf_s(elems[0].c_str(), "%G", & pConstL[0]);
		sscanf_s(elems[1].c_str(), "%G", & pConstL[1]);
		sscanf_s(elems[2].c_str(), "%G", & pConstL[2]);

		elems = split(lines[8], '\t');
		if (elems.size() != 3) {
			print_packet_error("Error in parameter 7", packet);
			return false;
		}

		sscanf_s(elems[0].c_str(), "%G", & pConstU[0]);
		sscanf_s(elems[1].c_str(), "%G", & pConstU[1]);
		sscanf_s(elems[2].c_str(), "%G", & pConstU[2]);

		elems = split(lines[9], '\t');
		if (elems.size() != 3) {
			print_packet_error("Error in parameter 8", packet);
			return false;
		}

		sscanf_s(elems[0].c_str(), "%G", & oConstL[0]);
		sscanf_s(elems[1].c_str(), "%G", & oConstL[1]);
		sscanf_s(elems[2].c_str(), "%G", & oConstL[2]);

		elems = split(lines[10], '\t');
		if (elems.size() != 3) {
			print_packet_error("Error in parameter 9", packet);
			return false;
		}

		sscanf_s(elems[0].c_str(), "%G", & oConstU[0]);
		sscanf_s(elems[1].c_str(), "%G", & oConstU[1]);
		sscanf_s(elems[2].c_str(), "%G", & oConstU[2]);

		elems = split(lines[11], '\t');
		if (elems.size() != 6) {
			print_packet_error("Error in parameter 10", packet);
			return false;
		}

		sscanf_s(elems[0].c_str(), "%d", & directionality[0]);
		sscanf_s(elems[1].c_str(), "%d", & directionality[1]);
		sscanf_s(elems[2].c_str(), "%d", & directionality[2]);
		sscanf_s(elems[3].c_str(), "%d", & directionality[3]);
		sscanf_s(elems[4].c_str(), "%d", & directionality[4]);
		sscanf_s(elems[5].c_str(), "%d", & directionality[5]);


		returnval = addSpringToShape(body_num, spring_num, max_force, dampingFactor, goalPos, goalOrient, pConstL, pConstU, oConstL, oConstU, directionality);

		stringstream out;
		out << (returnval ? "TRUE" : "FALSE") << "\n\n";
		string s = out.str();
		int size = s.size();
		returnval = send_all(socket, s.c_str(), &size);

	} else if (lines[0].compare("REMOVESPRING") == 0) {
		// params: spring_num

		if (lines.size() < 2) {
			print_packet_error("Incorrect number of parameters for command", packet);
			return false;
		}

		int spring_num;

		sscanf_s(lines[1].c_str(), "%d", & spring_num);

		returnval = removeSpring(spring_num);

		stringstream out;
		out << (returnval ? "TRUE" : "FALSE") << "\n\n";
		string s = out.str();
		int size = s.size();
		returnval = send_all(socket, s.c_str(), &size);

	} else if (lines[0].compare("SETGRAVITY") == 0) {
		// params = forces[3]
		
		if (lines.size() < 2) {
			print_packet_error("Incorrect number of parameters for command", packet);
			return false;
		}

		vector<string> elems = split(lines[1], '\t');
		if (elems.size() != 3) {
			print_packet_error("Error in parameter 2", packet);
			return false;
		}

		float forces[3];
		
		sscanf_s(elems[0].c_str(), "%G", & forces[0]);
		sscanf_s(elems[1].c_str(), "%G", & forces[1]);
		sscanf_s(elems[2].c_str(), "%G", & forces[2]);

		returnval = setGravity(forces);

		stringstream out;
		out << (returnval ? "TRUE" : "FALSE") << "\n\n";
		string s = out.str();
		int size = s.size();
		returnval = send_all(socket, s.c_str(), &size);


	} else if (lines[0].compare("APPLYFORCETOSHAPE") == 0) {
		// param: body_num, linear[3], torque[3]

		
		if (lines.size() < 4) {
			print_packet_error("Incorrect number of parameters for command", packet);
			return false;
		}

		int body_num;
		float linear[3];
		float torque[3];

		sscanf_s(lines[1].c_str(), "%d", & body_num);

		vector<string> elems = split(lines[2], '\t');
		if (elems.size() != 3) {
			print_packet_error("Error in parameter 2", packet);
			return false;
		}
		
		sscanf_s(elems[0].c_str(), "%G", & linear[0]);
		sscanf_s(elems[1].c_str(), "%G", & linear[1]);
		sscanf_s(elems[2].c_str(), "%G", & linear[2]);

		elems = split(lines[3], '\t');
		if (elems.size() != 3) {
			print_packet_error("Error in parameter 3", packet);
			return false;
		}
		
		sscanf_s(elems[0].c_str(), "%G", & torque[0]);
		sscanf_s(elems[1].c_str(), "%G", & torque[1]);
		sscanf_s(elems[2].c_str(), "%G", & torque[2]);

		returnval = applyForceToShape(body_num, linear, torque);

		stringstream out;
		out << (returnval ? "TRUE" : "FALSE") << "\n\n";
		string s = out.str();
		int size = s.size();
		returnval = send_all(socket, s.c_str(), &size);

	} else if (lines[0].compare("APPLYFORCE") == 0) {
		// params: falcon_num, force[3], time_in_secs

		if (lines.size() < 4) {
			print_packet_error("Incorrect number of parameters for command", packet);
			return false;
		}

		int falcon_num;
		float force[3];
		float time_in_sec;

		sscanf_s(lines[1].c_str(), "%d", & falcon_num);
		sscanf_s(lines[3].c_str(), "%G", & time_in_sec);

		vector<string> elems = split(lines[2], '\t');
		if (elems.size() != 3) {
			print_packet_error("Error in parameter 2", packet);
			return false;
		}
		
		sscanf_s(elems[0].c_str(), "%G", & force[0]);
		sscanf_s(elems[1].c_str(), "%G", & force[1]);
		sscanf_s(elems[2].c_str(), "%G", & force[2]);

		returnval = applyForce(falcon_num, force, time_in_sec);

		stringstream out;
		out << (returnval ? "TRUE" : "FALSE") << "\n\n";
		string s = out.str();
		int size = s.size();
		returnval = send_all(socket, s.c_str(), &size);

	} else {
		print_packet_error("Unknown command", packet);
		return false;
	}


	return returnval;


}

int __cdecl main(void) 
{
    WSADATA wsaData;
    int iResult;
	hasStarted = false;

    SOCKET ListenSocket = INVALID_SOCKET;
    SOCKET ClientSocket = INVALID_SOCKET;
	struct sockaddr_in client_info;
	int client_info_len;

    struct addrinfo *result = NULL;
    struct addrinfo hints;

    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed with error: %d\n", iResult);
        return 1;
    }

    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE;

    // Resolve the server address and port
    iResult = getaddrinfo(NULL, DEFAULT_PORT, &hints, &result);
    if ( iResult != 0 ) {
        printf("getaddrinfo failed with error: %d\n", iResult);
        WSACleanup();
        return 1;
    }

    // Create a SOCKET for connecting to server
    ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (ListenSocket == INVALID_SOCKET) {
        printf("socket failed with error: %ld\n", WSAGetLastError());
        freeaddrinfo(result);
        WSACleanup();
        return 1;
    }

    // Setup the TCP listening socket
    iResult = bind( ListenSocket, result->ai_addr, (int)result->ai_addrlen);
    if (iResult == SOCKET_ERROR) {
        printf("bind failed with error: %d\n", WSAGetLastError());
        freeaddrinfo(result);
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }

    freeaddrinfo(result);

	fd_set readfds;

	timeval timeout;
	timeout.tv_sec = 10;
	timeout.tv_usec = 0;


	while(1) {
		FD_ZERO(&readfds);

		iResult = listen(ListenSocket, SOMAXCONN);
		if (iResult == SOCKET_ERROR) {
			printf("listen failed with error: %d\n", WSAGetLastError());
			closesocket(ListenSocket);
			WSACleanup();
			return 1;
		}

		// Accept a client socket
		client_info_len =sizeof(client_info);
		ClientSocket = accept(ListenSocket, (struct sockaddr*)&client_info, &client_info_len);
		if (ClientSocket == INVALID_SOCKET) {
			printf("accept failed with error: %d\n", WSAGetLastError());
			continue;
		}

		packetbuf.clear();
		printf("Connection from %s, port %d\n", inet_ntoa(client_info.sin_addr), htons(client_info.sin_port)) ;

		// Receive until the peer shuts down the connection
		do {


			// Use select to implement a timeout


			FD_SET(ClientSocket, &readfds);

			iResult = select(NULL, &readfds, NULL, NULL, &timeout);

			if (iResult < 0) {
				printf("select failed with error: %d\n", WSAGetLastError());
			}
			
			if (iResult == 0) {
				printf("Timeout waiting for client, assuming it is dead\n");
			} else {

				if (FD_ISSET(ClientSocket, &readfds)) {
				
					string * packet;

					int returnval;
					returnval = recv_packet(ClientSocket, &packet);

					if (returnval < 0) {
						// recv had an error, forward out
						iResult = returnval;
					} else if ( returnval > 0) {
						// packet received!
//						cout << "Received: " << *packet << endl;
						bool sendresult = handle_packet(packet, ClientSocket);
						delete packet;

						if (!sendresult) {
							printf("Error sending packet to client, assuming it is dead\n");
							iResult = 0;
						}
					}
				}
			}

		} while (iResult > 0);

		// shutdown the connection since we're done
		iResult = shutdown(ClientSocket, SD_SEND);
		if (iResult == SOCKET_ERROR) {
			printf("shutdown failed with error: %d\n", WSAGetLastError());
		}

		// cleanup
		closesocket(ClientSocket);

		cout << "Client Disconnected" << endl;
	}

    // No longer need server socket
    closesocket(ListenSocket);

    WSACleanup();

    return 0;
}
