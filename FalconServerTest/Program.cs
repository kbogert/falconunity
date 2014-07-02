//  FalconServerTest
//
//  A small client to test the FalconServer program
//
/*  
    Created by Kenneth Bogert
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
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;

namespace FalconServerTest
{
    class Program
    {

        static string setSphereGodObjectPacket(int falcon_num, float radius, float mass, float [] pos, float minDistToMaxForce, float maxDistToMaxForce)
        {
            string returnval = "SETSPHEREGODOBJECT\n";
            returnval += falcon_num + "\n";
            returnval += radius + "\n";
            returnval += mass + "\n";
            for (int i = 0; i < 3; i++)
            {
                returnval += pos[i];
                if (i < 2)
                    returnval += "\t";
            }
            returnval += "\n";
            returnval += minDistToMaxForce + "\n";
            returnval += maxDistToMaxForce + "\n";


            return returnval;
        }
        
        static string setRigidBodyGodObjectPacket(int falcon_num, int body_num, float minDistToMaxForce, float maxDistToMaxForce)
        {
            string returnval = "SETRIGIDBODYGODOBJECT\n";
            returnval += falcon_num + "\n";
            returnval += body_num + "\n";
            returnval += minDistToMaxForce + "\n";
            returnval += maxDistToMaxForce + "\n";


            return returnval;
        }
        static string sendDynamicShape(int shape_num, float [] verts, int num_tris, float weight, float hardness, float [] startPos, float [] startOri, float [] lf, float [] af, float friction) {
            string returnval = "SENDDYNAMICSHAPE\n";
            
            returnval += shape_num + "\n";
            for (int i = 0; i < verts.Length; i++)
            {
                returnval += verts[i];
                if (i < verts.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";
            returnval += num_tris + "\n";
            returnval += weight + "\n";
            returnval += hardness + "\n";
            for (int i = 0; i < startPos.Length; i++)
            {
                returnval += startPos[i];
                if (i < startPos.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";
            for (int i = 0; i < startOri.Length; i++)
            {
                returnval += startOri[i];
                if (i < startOri.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";
            for (int i = 0; i < lf.Length; i++)
            {
                returnval += lf[i];
                if (i < lf.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";
            for (int i = 0; i < af.Length; i++)
            {
                returnval += af[i];
                if (i < af.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";

            returnval += friction + "\n";

            return returnval;

        }

        static string updateDynamicShape(int shape_num, float weight, float hardness, float[] lf, float[] af, float friction)
        {
            string returnval = "UPDATEDYNAMICSHAPE\n";

            returnval += shape_num + "\n";
            returnval += weight + "\n";
            returnval += hardness + "\n";

            for (int i = 0; i < lf.Length; i++)
            {
                returnval += lf[i];
                if (i < lf.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";
            for (int i = 0; i < af.Length; i++)
            {
                returnval += af[i];
                if (i < af.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";

            returnval += friction + "\n";

            return returnval;

        }

        static string applyForceToShape(int shape_num, float[] linear, float[] torque)
        {
            string returnval = "APPLYFORCETOSHAPE\n";

            returnval += shape_num + "\n";
            for (int i = 0; i < linear.Length; i++)
            {
                returnval += linear[i];
                if (i < linear.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";
            for (int i = 0; i < torque.Length; i++)
            {
                returnval += torque[i];
                if (i < torque.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";

            return returnval;
        }

        static string setDynamicShape(int shape_num, float[] pos, float[] ori)
        {
            string returnval = "SETDYNAMICSHAPE\n";

            returnval += shape_num + "\n";
            for (int i = 0; i < pos.Length; i++)
            {
                returnval += pos[i];
                if (i < pos.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";
            for (int i = 0; i < ori.Length; i++)
            {
                returnval += ori[i];
                if (i < ori.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";

            return returnval;
        }

        static string applyForceToGod(int falcon, float[] force, float time)
        {
            string returnval = "APPLYFORCE\n";

            returnval += falcon + "\n";
            for (int i = 0; i < force.Length; i++)
            {
                returnval += force[i];
                if (i < force.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";
            returnval += time + "\n";

            return returnval;
        }


        static string setForceField(int falcon, float[] force)
        {
            string returnval = "SETFORCEFIELD\n";

            returnval += falcon + "\n";
            for (int i = 0; i < force.Length; i++)
            {
                returnval += force[i];
                if (i < force.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";

            return returnval;
        }

        static string removeGodObject(int falcon_num)
        {
            string returnval = "REMOVEGODOBJECT\n";

            returnval += falcon_num + "\n";

            return returnval;
        }


        static string removeSpring(int spring_num)
        {
            string returnval = "REMOVESPRING\n";

            returnval += spring_num + "\n";

            return returnval;
        }


        static string removeDynamicShape(int shape_num)
        {
            string returnval = "REMOVEDYNAMICSHAPE\n";

            returnval += shape_num + "\n";

            return returnval;
        }


        static string setGravity(float[] force)
        {
            string returnval = "SETGRAVITY\n";

            for (int i = 0; i < force.Length; i++)
            {
                returnval += force[i];
                if (i < force.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";

            return returnval;
        }

        static string updatePacket() {
            return "UPDATE\n";
        }

        static void parseUpdatePacket(string packet, out float fps, out float[] tipPos, out float[] godPos, out bool[] buttons, out float[] forces, out float[] objPose)
        {
            
            string [] lines = packet.Split('\n');

            tipPos = null;
            godPos = null;
            buttons = null;
            forces = null;
            objPose = null;
            try
            {
                fps = float.Parse(lines[1]);
            }
            catch (FormatException)
            {
                fps = float.PositiveInfinity;
            }

            int numFalcons = int.Parse(lines[2]);
            for (int f = 0; f < numFalcons; f++)
            {

                string[] elems = lines[3 + f * 4].Split('\t');
                float[] t = new float[elems.Length];
                for (int i = 0; i < elems.Length; i++)
                {
                    t[i] = float.Parse(elems[i]);
                }
                tipPos = t;

                elems = lines[4 + f * 4].Split('\t');
                t = new float[elems.Length];
                for (int i = 0; i < elems.Length; i++)
                {
                    t[i] = float.Parse(elems[i]);
                }
                godPos = t;

                elems = lines[5 + f * 4].Split('\t');
                t = new float[elems.Length];
                for (int i = 0; i < elems.Length; i++)
                {
                    t[i] = float.Parse(elems[i]);
                }
                forces = t;

                elems = lines[6 + f * 4].Split('\t');
                bool[] b = new bool[elems.Length];
                for (int i = 0; i < elems.Length; i++)
                {
                    b[i] = int.Parse(elems[i]) == 1;
                }
                buttons = b;

            }

            int curPos = 3 + numFalcons * 4;
            int numObjects = int.Parse(lines[curPos]);


            for (int f = 0; f < numObjects; f++)
            {
                string[] elems = lines[curPos + 1 + f].Split('\t');
                objPose = new float[elems.Length];
                for (int i = 0; i < elems.Length; i++)
                {
                    objPose[i] = float.Parse(elems[i]);
                }
            }
        }


        static string addSpring(int body_id, int spring_id, float max_force, float dampingFactor, float[] pos, float[] ori, float[] minPos, float[] maxPos, float[] minOri, float[] maxOri, int[] directionality)
        {
            string returnval = "ADDSPRINGTOSHAPE\n";

            returnval += body_id + "\n";
            returnval += spring_id + "\n";
            returnval += max_force + "\n";
            returnval += dampingFactor + "\n";

            for (int i = 0; i < pos.Length; i++)
            {
                returnval += pos[i];
                if (i < pos.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";

            for (int i = 0; i < ori.Length; i++)
            {
                returnval += ori[i];
                if (i < ori.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";

            for (int i = 0; i < minPos.Length; i++)
            {
                returnval += minPos[i];
                if (i < minPos.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";

            for (int i = 0; i < maxPos.Length; i++)
            {
                returnval += maxPos[i];
                if (i < maxPos.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";

            for (int i = 0; i < minOri.Length; i++)
            {
                returnval += minOri[i];
                if (i < minOri.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";

            for (int i = 0; i < maxOri.Length; i++)
            {
                returnval += maxOri[i];
                if (i < maxOri.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";

            for (int i = 0; i < directionality.Length; i++)
            {
                returnval += directionality[i];
                if (i < directionality.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";

            return returnval;
        }

        static string lerpSpring(int spring_id, float max_force, float dampingFactor, float[] pos, float[] ori, float[] minPos, float[] maxPos, float[] minOri, float[] maxOri, int[] directionality, float time_in_secs)
        {
            string returnval = "";

            returnval += "S\n";
            returnval += spring_id + "\n";
            returnval += max_force + "\n";
            returnval += dampingFactor + "\n";

            for (int i = 0; i < pos.Length; i++)
            {
                returnval += pos[i];
                if (i < pos.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";

            for (int i = 0; i < ori.Length; i++)
            {
                returnval += ori[i];
                if (i < ori.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";

            for (int i = 0; i < minPos.Length; i++)
            {
                returnval += minPos[i];
                if (i < minPos.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";

            for (int i = 0; i < maxPos.Length; i++)
            {
                returnval += maxPos[i];
                if (i < maxPos.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";

            for (int i = 0; i < minOri.Length; i++)
            {
                returnval += minOri[i];
                if (i < minOri.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";

            for (int i = 0; i < maxOri.Length; i++)
            {
                returnval += maxOri[i];
                if (i < maxOri.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";

            for (int i = 0; i < directionality.Length; i++)
            {
                returnval += directionality[i];
                if (i < directionality.Length - 1)
                    returnval += "\t";
            }

            returnval += "\n";

            returnval += time_in_secs + "\n";

            return returnval;
        }


        static string updateHapticTip(int falcon_num, float[] pos, float[] ori, float[] scale, bool useCompensator, float time_in_secs)
        {
            string returnval = "UPDATE\n";

            returnval += "H\n";
            returnval += falcon_num + "\n";

            for (int i = 0; i < pos.Length; i++)
            {
                returnval += pos[i];
                if (i < pos.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";

            for (int i = 0; i < ori.Length; i++)
            {
                returnval += ori[i];
                if (i < ori.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";

            for (int i = 0; i < scale.Length; i++)
            {
                returnval += scale[i];
                if (i < scale.Length - 1)
                    returnval += "\t";
            }
            returnval += "\n";
            returnval += (useCompensator ? "1" : "0") + "\n";
            returnval += time_in_secs + "\n";

            return returnval;
        }

        static void sendPacket(Socket sender, string data)
        {
            byte [] toSend = Encoding.ASCII.GetBytes(data + "\n\n");

            int bytesSent = 0;
            int sent = 0;
            do
            {
                sent = sender.Send(toSend);
                bytesSent += sent;

            } while (bytesSent < toSend.Length && sent > 0); 

        }

        static string recvPacket(Socket sock)
        {
            byte[] buf = new byte[1024];
            string returnval = "";

            do
            {
                int bytesRec = sock.Receive(buf);
                string temp = Encoding.ASCII.GetString(buf, 0, bytesRec);
                returnval += temp;
            } while (returnval.IndexOf("\n\n") < 0);

            return returnval.Substring(0, returnval.IndexOf("\n\n") + 1);
        }

        static void Main(string[] args)
        {

            // create socket connection to falcon server
            string url = "localhost";

            IPHostEntry ipHostInfo = Dns.Resolve(url);
            IPAddress ipAddress = ipHostInfo.AddressList[0];
            IPEndPoint remoteEP = new IPEndPoint(ipAddress,8747);

            // Create a TCP/IP  socket.
            Socket sender = new Socket(AddressFamily.InterNetwork, 
                SocketType.Stream, ProtocolType.Tcp );

            try {
                sender.Connect(remoteEP);

                Console.WriteLine("Socket connected to {0}",
                    sender.RemoteEndPoint.ToString());

                sendPacket(sender, "START\n");
                string result = recvPacket(sender);

                sendPacket(sender, "START\n");
                result = recvPacket(sender);
                Console.WriteLine("Sent START, received {0}",result);

                // add god object
                string msg = setSphereGodObjectPacket(0, .1f, .001f, new float[] { 0, 0, 0 }, 0.05f, 0.15f);

                sendPacket(sender, msg);
                result = recvPacket(sender);

                Console.WriteLine("Sent GOD object, received {0}",
                    result);


                // send over simple shape
                float [] verts = {-20, 0, 20,
                     -20, 0, -20,
                    20, 0, 20,
                    20, 0, 20,
                    -20,0,-20,
                    20,0,-20};
                
                float [] startPos = {0,-0.025f,0};
	            float [] startOri = {0,0,0,1};

                float [] lf = {0,1,0};
                float [] af = {0,0,0};

                msg = sendDynamicShape(0, verts, 2, 1, 1, startPos, startOri, lf, af, 1);

                sendPacket(sender, msg);
                result = recvPacket(sender);

                Console.WriteLine("Sent dynamic object, received {0}",
                    result);


                // apply force to shape

                msg = applyForceToShape(0, new float[] { 0, 1, 0 }, new float[] { 1, 0, 0 });

                sendPacket(sender, msg);
                result = recvPacket(sender);

                Console.WriteLine("Apply Force to object, received {0}",
                    result);

                // apply force to haptic tip
                msg = applyForceToGod(0, new float[] { 0, 9, 0 }, .1f);
                sendPacket(sender, msg);
                result = recvPacket(sender);

                Console.WriteLine("Apply Force to GOD, received {0}",
                    result);

                msg = setRigidBodyGodObjectPacket(0, 0, 0.05F, 0.15F);
                sendPacket(sender, msg);
                result = recvPacket(sender);

                Console.WriteLine("Set God to Rigid Body, received {0}",
                    result);

                // print out god object and haptic tip positions until button is pressed
                Thread.Sleep((int)(1.0  * 1000));

                while (true)
                {
                    Thread.Sleep((int)(1.0 / 60.0 * 1000));

                    msg = updateHapticTip(0, new float[]{0, 1, 0}, new float[] {0,0,0,1}, new float[] {1,1,1}, true, 1.0f/60.0f);
                    sendPacket(sender, msg);
                    result = recvPacket(sender);

                    float fps;
                    float[] tipPos;
                    float[] godPos;
                    float[] forces;
                    bool[] buttons;
                    float[] objPose;

                    parseUpdatePacket(result, out fps, out tipPos, out godPos, out buttons, out forces, out objPose);

                    if (tipPos == null)
                        break;

                    Console.WriteLine("FPS: {0} Tip: ({1}, {2}, {3}) God ({4}, {5}, {6}) Forces ({7}, {8}, {9})", fps, tipPos[0], tipPos[1], tipPos[2], godPos[0], godPos[1], godPos[2], forces[0], forces[1], forces[2]);

                    
                    if (buttons[0] || buttons[1] || buttons[2] || buttons[3])
                        break;



                }
                // disconnect, reconnect

                sender.Shutdown(SocketShutdown.Both);
                sender.Close();

                Thread.Sleep((int)(.5 * 1000));

                sender = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
                sender.Connect(remoteEP);

                msg = setSphereGodObjectPacket(0, .1f, .001f, new float[] { 0, 0, 0 }, 0.05f, 0.15f);

                sendPacket(sender, msg);
                result = recvPacket(sender);

                Console.WriteLine("Sent GOD object, received {0}",
                    result);

                msg = removeGodObject(0);

                sendPacket(sender, msg);
                result = recvPacket(sender);

                Console.WriteLine("Remove GOD object, received {0}",
                    result); 


                // add a spring
               	float []maxPos = {0,10,0};
	            float []minPos = {0,-10,0};
	            float []maxOri = {0,0,0};
            	float []minOri = {0,0,0};
                int[] dir = { 0,0,0,0,0,0};

                msg = addSpring(0, 0, 1, 1, startPos, startOri, minPos, maxPos, minOri, maxOri, dir);
                sendPacket(sender, msg);
                result = recvPacket(sender);

                Console.WriteLine("Add Spring, received {0}",
                    result);

                Thread.Sleep((int)(1 * 1000));

                // lerp the spring until button is pressed

                while (true)
                {
                    Thread.Sleep((int)(1.0 / 60.0 * 1000));

                    startPos[1] += .01f;
                    maxPos[1] += .01f;
                    minPos[1] += .01f;

                    msg = updateHapticTip(0, new float[] { 0, 1, 0 }, new float[] { 0, 0, 0, 1 }, new float[] { 1, 1, 1 }, false, 1.0f / 60.0f) + lerpSpring(0, 1, 1, startPos, startOri, maxPos, minPos, minOri, maxOri, dir, 1.0f / 60.0f);
                    sendPacket(sender, msg);
                    result = recvPacket(sender);

                    float fps;
                    float[] tipPos;
                    float[] godPos;
                    float[] forces;
                    bool[] buttons;
                    float[] objPose;

                    parseUpdatePacket(result, out fps, out tipPos, out godPos, out buttons, out forces, out objPose);

                    if (tipPos == null)
                        break;

                    Console.WriteLine("FPS: {0} Tip: ({1}, {2}, {3}) God ({4}, {5}, {6}) Forces ({7}, {8}, {9})", fps, tipPos[0], tipPos[1], tipPos[2], godPos[0], godPos[1], godPos[2], forces[0], forces[1], forces[2]);


                    if (buttons[0] || buttons[1] || buttons[2] || buttons[3])
                        break;



                }

                // call all other methods (keepalive, set gravity, apply force to shape, set shape pose, remove spring, remove object

                sendPacket(sender, "KEEPALIVE\n");
                result = recvPacket(sender);
                Console.WriteLine("Keepalive, received {0}",
                    result);

                /*
                 * geterror
                 * setforcefield
                 * removedynamicshape
                 * updateDynamicShape
                 * setDynamicShape
                 * setgravity
                 * removeSpring
                 */

                sendPacket(sender, "GETERROR\n");
                result = recvPacket(sender);
                Console.WriteLine("Geterror, received {0}",
                    result);


                sendPacket(sender, setForceField(0, new float[] {0,9,0}));
                result = recvPacket(sender);
                Console.WriteLine("setForceField, received {0}",
                    result);


                sendPacket(sender, updateDynamicShape(0, 1, 1, lf, af, 1));
                result = recvPacket(sender);
                Console.WriteLine("updateDynamicShape, received {0}",
                    result);


                sendPacket(sender, setDynamicShape(0, startPos, startOri));
                result = recvPacket(sender);
                Console.WriteLine("setDynamicShape, received {0}",
                    result);



                sendPacket(sender, setGravity(new float[] { 0, -9, 0 }));
                result = recvPacket(sender);
                Console.WriteLine("setGravity, received {0}",
                    result);


                sendPacket(sender, removeSpring(0));
                result = recvPacket(sender);
                Console.WriteLine("removeSpring, received {0}",
                    result);

                sendPacket(sender, removeDynamicShape(0));
                result = recvPacket(sender);
                Console.WriteLine("removeDynamicShape, received {0}",
                    result);


                sendPacket(sender, "STOP\n");
                result = recvPacket(sender);

                // disconnect


                // Release the socket.
                sender.Shutdown(SocketShutdown.Both);
                sender.Close();
                
            } catch (ArgumentNullException ane) {
                Console.WriteLine("ArgumentNullException : {0}",ane.ToString());
            } catch (SocketException se) {
                Console.WriteLine("SocketException : {0}",se.ToString());
            } catch (Exception e) {
                Console.WriteLine("Unexpected exception : {0}", e.ToString());
            }

        }
    }
}
