#define USE_REMOTE
 
using UnityEngine;
using System.Collections;
using System.Text;


#if USE_REMOTE
using System.Net;
using System.Net.Sockets;
using System;
using System.Threading;

public class FalconUnityCall {

	public static bool IsRemote() {
		return true;
	}
	
	private static string serverAddress;
	private static int serverPort;
	private static Socket serverSocket;
	
	private static float lastFPS;
	private static Hashtable falconData;
	private static Hashtable objData;
	private static Hashtable springUpdates;
	private static Hashtable falconUpdates;
	
	private static Queue packetQueue;
	private static Queue inPacketQueue;
	private static ManualResetEvent queueEvent;
	private static Thread clientThread;
	
	private static bool initialized;
	
	private static int numfalcons = -1;
	private static int updatePacketMinSize = 3;
	
	
	private enum ConnectionState {
		Disconnected,
		Idle,
		Connecting,
		Sending,
		Receiving
	};
	
	private static ConnectionState connectionState = ConnectionState.Disconnected;
	private static object connectionStateLock = new object();
	private static string packetCache;
	
	
	private static ConnectionState getCurrentState() {
		lock (connectionStateLock) {
			return connectionState;
		}
	}
	
	private static void setCurrentState(ConnectionState c) {
		lock (connectionStateLock) {
			connectionState = c;
		}
	}
	
	public static void setServerParams(string address, int port) {
		serverAddress = address;
		serverPort = port;
		
		falconData = new Hashtable();
		objData = new Hashtable();
		springUpdates = new Hashtable();
		falconUpdates = new Hashtable();
		
		packetQueue = Queue.Synchronized(new Queue());
		inPacketQueue = Queue.Synchronized(new Queue());
		queueEvent = new ManualResetEvent(false);
		
		clientThread = new Thread(new ThreadStart(socketThread));
    	clientThread.Start();
		
		initialized = true;
	}
	
	// Start, Stop, Update, GetLastError are synchronous and immediate, all other methods place packets into a queue to be transmitted on next call to Update()
	
	// called ONCE to initialize falcons
	public static void Start() {
		if (initialized) {
			packetQueue.Enqueue("START\n");
			queueEvent.Set();
			return;
		}
		throw new Exception("Call setServerParams to initialize this class before anything else");
		
	} //returns number of falcons
	
	public static int getNumFalcons() {
		return numfalcons;
	}
	
	// called to stop falcons and clean up physics world
    public static bool Stop() {
		if (initialized) {
			
			packetQueue.Clear();
			
			packetQueue.Enqueue("STOP\n");
			packetQueue.Enqueue("STOP\n");
			queueEvent.Set();
			
			
			int counter = 0;
			while ((packetQueue.Count > 0 || getCurrentState() != ConnectionState.Idle) && counter < 150) {
				Update();
				Thread.Sleep(2);
				counter ++;
			}
			
			clientThread.Abort();
			
			return true;
		}
		throw new Exception("Call setServerParams to initialize this class before anything else");
	
	}
	
	public static void disconnect() {
		if (serverSocket != null && serverSocket.Connected) {
			serverSocket.Disconnect(false);
		}
		serverSocket = null;
		
	}
	
	// should be called once a frame
	public static bool Update() {
		
		// generate update packet
		// send and parse results

		if (initialized) {
			
			while (inPacketQueue.Count > 0) {
				parseUpdatePacket((string)inPacketQueue.Dequeue());
			}
			
			ConnectionState cs = getCurrentState();
			
			if (cs != ConnectionState.Disconnected && cs != ConnectionState.Connecting) {
				string packet = "UPDATE\n";
				foreach (object o in falconUpdates.Values) {
					string line = (string) o;
					packet += line;
				}
				
				falconUpdates.Clear();
				
				foreach (object o in springUpdates.Values) {
					string line = (string) o;
					packet += line;
				}
				
				springUpdates.Clear();
				
				packetQueue.Enqueue(packet);
				queueEvent.Set();

			}
			return true;
		}
		
		throw new Exception("Call setServerParams to initialize this class before anything else");
	}
	
	// returns physics world frames per second
    public static float getFPS() {
		return lastFPS;
	}
	
	// returns the position of the haptic tip
    public static bool getTipPosition(int falcon_num, out Vector3 pos_out) {
		if (falconData.ContainsKey(falcon_num)) {
			pos_out = ((FalconDataStruct)falconData[falcon_num]).tipPosition;
			return true;
		}
		pos_out = new Vector3();
		return false;
	}
	
	// returns the position of the god object, note that this is a physics-simulated rigid body
    public static bool getGodPosition(int falcon_num, out Vector3 pos_out) {
		if (falconData.ContainsKey(falcon_num)) {
			pos_out = ((FalconDataStruct)falconData[falcon_num]).godPosition;
			return true;
		}
		pos_out = new Vector3();
		return false;		
	}
	
	// returns the averaged forces the falcon is generating
    public static bool getFalconForces(int falcon_num, out Vector3 force_out) {
		if (falconData.ContainsKey(falcon_num)) {
			force_out = ((FalconDataStruct)falconData[falcon_num]).curforces;
			return true;
		}
		force_out = new Vector3();
		return false;		
	}
	
	// apply a static force to the haptic tip every tick
    public static void setForceField(int falcon_num, Vector3 force) {
        string returnval = "SETFORCEFIELD\n";

        returnval += falcon_num + "\n";
        for (int i = 0; i < 3; i++)
        {
            returnval += force[i];
            if (i < 2)
                returnval += "\t";
        }
        returnval += "\n";

		packetQueue.Enqueue(returnval);
		queueEvent.Set();
	}
	
	// apply a force to the god object for a specified amount of time
	public static void applyForce(int falcon_num, Vector3 force, float time_in_secs) {
        string returnval = "APPLYFORCE\n";

        returnval += falcon_num + "\n";
        for (int i = 0; i < 3; i++)
        {
            returnval += force[i];
            if (i < 2)
                returnval += "\t";
        }
        returnval += "\n";
        returnval += time_in_secs + "\n";

		packetQueue.Enqueue(returnval);
		queueEvent.Set();
	}

	// set the gravity force applied to all objects
	public static void setGravity(Vector3 force) {
        string returnval = "SETGRAVITY\n";

        for (int i = 0; i < 3; i++)
        {
            returnval += force[i];
            if (i < 2)
                returnval += "\t";
        }
        returnval += "\n";

		packetQueue.Enqueue(returnval);
		queueEvent.Set();
	}
	
	
    public static string getLastError() {
		throw new Exception("Method not implemented over the network.");
	} 
   
	// initialize god object to a sphere.  Calling this again will delete old god object and replace it with a new one
	public static void setSphereGodObject(int falcon_num, float radius, float mass, Vector3 pos, float minDist, float maxDist) {
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
        returnval += minDist + "\n";
        returnval += maxDist + "\n";
	
		packetQueue.Enqueue (returnval);
		queueEvent.Set();
	}
	
	public static void setRigidBodyGodObject(int falcon_num, int body_num, float minDist, float maxDist) {
		string returnval = "SETRIGIDBODYGODOBJECT\n";
        returnval += falcon_num + "\n";
        returnval += body_num + "\n";
        returnval += minDist + "\n";
        returnval += maxDist + "\n";
	
		packetQueue.Enqueue (returnval);
		queueEvent.Set();
	}    

	public static void removeGodObject(int falcon_num) {
		string returnval = "REMOVEGODOBJECT\n";

		returnval += falcon_num + "\n";

		packetQueue.Enqueue (returnval);
		queueEvent.Set();
	}    

	// gets the states of the falcon's buttons
	public static bool getFalconButtonStates(int falcon_num, out bool[] buttons) {
		if (falconData.ContainsKey(falcon_num)) {
			buttons = ((FalconDataStruct)falconData[falcon_num]).buttons;
			return true;
		}
		buttons = new bool[0];
		return false;			
	}
		
	// place a concave shape in the physics world with the following parameters 
	public static void sendDynamicShape(int body_num, float [] shape, int num_tris, float weight, float k, Vector3 startPos, Quaternion startOrient, Vector3 linearFactors, Vector3 angularFactors, float friction ) {
 		
		StringBuilder sb = new StringBuilder();
		sb.Append("SENDDYNAMICSHAPE\n");
            
        sb.Append(body_num).Append("\n");
        for (int i = 0; i < shape.Length; i++)
        {
            sb.Append(shape[i]);
            if (i < shape.Length - 1)
                sb.Append("\t");
        }
        sb.Append("\n");
		
		string returnval = sb.ToString();
        returnval += num_tris + "\n";
        returnval += weight + "\n";
        returnval += k + "\n";
        for (int i = 0; i < 3; i++)
        {
            returnval += startPos[i];
            if (i < 2)
                returnval += "\t";
        }
        returnval += "\n";
        for (int i = 0; i < 4; i++)
        {
            returnval += startOrient[i];
            if (i < 3)
                returnval += "\t";
        }
        returnval += "\n";
        for (int i = 0; i < 3; i++)
        {
            returnval += linearFactors[i];
            if (i < 2)
                returnval += "\t";
        }
        returnval += "\n";
        for (int i = 0; i < 3; i++)
        {
            returnval += angularFactors[i];
            if (i < 2)
                returnval += "\t";
        }
        returnval += "\n";

        returnval += friction + "\n";	
		
		packetQueue.Enqueue(returnval);
		queueEvent.Set();
	}
	
	// remove the given shape from the physics world
	public static void removeDynamicShape(int body_num) {
        string returnval = "REMOVEDYNAMICSHAPE\n";

        returnval += body_num + "\n";

		packetQueue.Enqueue(returnval);
		queueEvent.Set();
	}
	
	// get the pose of the given shape
	public static bool getDynamicShapePose(int body_num, out Vector3 pos, out Quaternion orient) {
		if (objData.ContainsKey(body_num)) {
			pos = ((DynObjStruct)objData[body_num]).pos;
			orient = ((DynObjStruct)objData[body_num]).orient;
			return true;
		}
		pos = new Vector3();
		orient = new Quaternion();
		return false;
	}
	
	// set the pose of the given shape *** WARNING *** resets the shape's velocity
	public static void setDynamicShapePose(int body_num, Vector3 pos, Quaternion orient) {
	
        string returnval = "SETDYNAMICSHAPE\n";

        returnval += body_num + "\n";
        for (int i = 0; i < 3; i++)
        {
            returnval += pos[i];
            if (i < 2)
                returnval += "\t";
        }
        returnval += "\n";
        for (int i = 0; i < 4; i++)
        {
            returnval += orient[i];
            if (i < 3)
                returnval += "\t";
        }
        returnval += "\n";
		
		packetQueue.Enqueue(returnval);
		queueEvent.Set();
	}
	
	// apply force to the given shape
	public static void applyForceToShape(int body_num, Vector3 linear, Vector3 torque) {
	
        string returnval = "APPLYFORCETOSHAPE\n";

        returnval += body_num + "\n";
        for (int i = 0; i < 3; i++)
        {
            returnval += linear[i];
            if (i < 2)
                returnval += "\t";
        }
        returnval += "\n";
        for (int i = 0; i < 3; i++)
        {
            returnval += torque[i];
            if (i < 2)
                returnval += "\t";
        }
        returnval += "\n";

		packetQueue.Enqueue(returnval);
		queueEvent.Set();
	}
	
	// update the properties of the given shape
	public static void updateDynamicShape(int body_num, float weight, float hardness, Vector3 linearFactors, Vector3 angularFactors, float friction ) {
        string returnval = "UPDATEDYNAMICSHAPE\n";

        returnval += body_num + "\n";
        returnval += weight + "\n";
        returnval += hardness + "\n";

        for (int i = 0; i < 3; i++)
        {
            returnval += linearFactors[i];
            if (i < 2)
                returnval += "\t";
        }
        returnval += "\n";
        for (int i = 0; i < 3; i++)
        {
            returnval += angularFactors[i];
            if (i < 2)
                returnval += "\t";
        }
        returnval += "\n";

        returnval += friction + "\n";

		packetQueue.Enqueue(returnval);
		queueEvent.Set();
	}
	
	
	// attach a 6 dimensional damped spring to an existing concave shape
	public static void addSpringToShape(int body_num, int spring_num, float max_force, float dampingFactor, Vector3 goalPos, Quaternion goalOrient, Vector3 posConstraintLower, Vector3 posConstraintUpper, Vector3 orientConstraintLower, Vector3 orientConstraintUpper, int [] directionality) {
        string returnval = "ADDSPRINGTOSHAPE\n";

        returnval += body_num + "\n";
        returnval += spring_num + "\n";
        returnval += max_force + "\n";
        returnval += dampingFactor + "\n";		

        for (int i = 0; i < 3; i++)
        {
            returnval += goalPos[i];
            if (i < 2)
                returnval += "\t";
        }
        returnval += "\n";

        for (int i = 0; i < 4; i++)
        {
            returnval += goalOrient[i];
            if (i < 3)
                returnval += "\t";
        }
        returnval += "\n";

        for (int i = 0; i < 3; i++)
        {
            returnval += posConstraintLower[i];
            if (i < 2)
                returnval += "\t";
        }
        returnval += "\n";

        for (int i = 0; i < 3; i++)
        {
            returnval += posConstraintUpper[i];
            if (i < 2)
                returnval += "\t";
        }
        returnval += "\n";

        for (int i = 0; i < 3; i++)
        {
            returnval += orientConstraintLower[i];
            if (i < 2)
                returnval += "\t";
        }
        returnval += "\n";

        for (int i = 0; i < 3; i++)
        {
            returnval += orientConstraintUpper[i];
            if (i < 2)
                returnval += "\t";
        }
        returnval += "\n";

        for (int i = 0; i < 6; i++)
        {
            returnval += directionality[i];
            if (i < 5)
                returnval += "\t";
        }
        returnval += "\n";

		packetQueue.Enqueue(returnval);
		queueEvent.Set();
	}
	
	// remove the specified spring from its shape, does not remove the shape itself
	public static void removeSpring(int spring_num) {
        string returnval = "REMOVESPRING\n";

        returnval += spring_num + "\n";
		
		packetQueue.Enqueue(returnval);
		queueEvent.Set();

	}
	
	// move the pose of the spring and set parameters
	public static void setSpring(int spring_num, float max_force, float dampingFactor, Vector3 goalPos, Quaternion goalOrient, Vector3 posConstraintLower, Vector3 posConstraintUpper, Vector3 orientConstraintLower, Vector3 orientConstraintUpper, int [] directionality) {
		lerpSpring(spring_num, max_force, dampingFactor, goalPos, goalOrient, posConstraintLower, posConstraintUpper, orientConstraintLower, orientConstraintUpper, directionality, 0);
	}
	
	// interpolate the pose and parameters of the given spring over the given time period.  interpolation is done at each physics tick
	public static void lerpSpring(int spring_num, float max_force, float dampingFactor, Vector3 goalPos, Quaternion goalOrient, Vector3 posConstraintLower, Vector3 posConstraintUpper, Vector3 orientConstraintLower, Vector3 orientConstraintUpper, int [] directionality, float time_in_secs) {
        string returnval = "S\n";
		
        returnval += spring_num + "\n";
        returnval += max_force + "\n";
        returnval += dampingFactor + "\n";

        for (int i = 0; i < 3; i++)
        {
            returnval += goalPos[i];
            if (i < 2)
                returnval += "\t";
        }
        returnval += "\n";

        for (int i = 0; i < 4; i++)
        {
            returnval += goalOrient[i];
            if (i < 3)
                returnval += "\t";
        }
            returnval += "\n";

        for (int i = 0; i < 3; i++)
        {
            returnval += posConstraintLower[i];
            if (i < 2)
                returnval += "\t";
        }
        returnval += "\n";

        for (int i = 0; i < 3; i++)
        {
            returnval += posConstraintUpper[i];
            if (i < 2)
                returnval += "\t";
        }
        returnval += "\n";

        for (int i = 0; i < 3; i++)
        {
            returnval += orientConstraintLower[i];
            if (i < 2)
                returnval += "\t";
        }
        returnval += "\n";

        for (int i = 0; i < 3; i++)
        {
            returnval += orientConstraintUpper[i];
            if (i < 2)
                returnval += "\t";
        }
        returnval += "\n";

		for (int i = 0; i < 6; i++)
        {
            returnval += directionality[i];
            if (i < 5)
                returnval += "\t";
        }
        returnval += "\n";

        returnval += time_in_secs + "\n";
		
		springUpdates[spring_num] = returnval;
	}
	
	
	public static void updateHapticTransform(int falcon_num, Vector3 position, Quaternion rotation, Vector3 scale, bool useCompensator, float time_in_secs) {
        string returnval = "H\n";
		
        returnval += falcon_num + "\n";

        for (int i = 0; i < 3; i++)
        {
            returnval += position[i];
            if (i < 2)
                returnval += "\t";
        }
        returnval += "\n";

        for (int i = 0; i < 4; i++)
        {
            returnval += rotation[i];
            if (i < 3)
                returnval += "\t";
        }
            returnval += "\n";

        for (int i = 0; i < 3; i++)
        {
            returnval += scale[i];
            if (i < 2)
                returnval += "\t";
        }
        returnval += "\n";
		returnval += (useCompensator ? "1" : "0") + "\n";
        returnval += time_in_secs + "\n";
		
		falconUpdates[falcon_num] = returnval;
		
	}	


    private static void parseUpdatePacket(string packet)
    {
        try {
        	string [] lines = packet.Split('\n');
			
			if (lines.Length <= updatePacketMinSize) {
				// Not an update packet	
				return;
			}
			
        	try
        	{
        	    lastFPS = float.Parse(lines[1]);
        	}
        	catch (FormatException )
        	{
        	    lastFPS = float.PositiveInfinity;
        	}
        	
        	numfalcons = int.Parse(lines[2]);
        	for (int f = 0; f < numfalcons; f ++) {
        	
        	FalconDataStruct data = new FalconDataStruct();
        	    string[] elems = lines[3 + f * 4].Split('\t');
        	    float[] t = new float[elems.Length];
        	    for (int i = 0; i < elems.Length; i++)
        	    {
        	        t[i] = float.Parse(elems[i]);
        	    }
        	    data.tipPosition = new Vector3(t[0], t[1], t[2]);
        	
        	    elems = lines[4 + f * 4].Split('\t');
        	    t = new float[elems.Length];
        	    for (int i = 0; i < elems.Length; i++)
        	    {
        	        t[i] = float.Parse(elems[i]);
        	    }
        	    data.godPosition = new Vector3(t[0], t[1], t[2]);
        	
        	    elems = lines[5 + f * 4].Split('\t');
        	    t = new float[elems.Length];
        	    for (int i = 0; i < elems.Length; i++)
        	    {
        	        t[i] = float.Parse(elems[i]);
        	    }
        	    data.curforces = new Vector3(t[0], t[1], t[2]);
        	
        	    elems = lines[6 + f * 4].Split('\t');
        	    bool [] b = new bool[elems.Length];
        	    for (int i = 0; i < elems.Length; i++)
        	    {
        	        b[i] = int.Parse(elems[i]) == 1;
        	    }
        	    data.buttons = b;
        	
        	falconData[f] = data;
        	}
        	
        	int curPos = 3 + numfalcons * 4;
        	int numObjects = int.Parse(lines[curPos]);
        	
        	
        	for (int f = 0; f < numObjects; f ++) {
        	    string [] elems = lines[curPos + 1 + f].Split('\t');
        	    float [] objPose = new float[elems.Length];
        	    for (int i = 0; i < elems.Length; i++)
        	    {
        	        objPose[i] = float.Parse(elems[i]);
        	    }
        	DynObjStruct d = new DynObjStruct();
        	d.pos = new Vector3(objPose[1], objPose[2], objPose[3]);
        	d.orient = new Quaternion(objPose[4], objPose[5], objPose[6], objPose[7]);
        	
        	objData[(int)objPose[0]] = d;
        	}
        } catch (Exception ex) {
        	Debug.LogException(ex);
			Debug.LogError(packet);
        }
		


    }


	private static void tryConnect() {
		if (!initialized) 
			return;
		
		if (serverSocket != null) {
			
			// we need to disconnect the current connection;
			try {
				serverSocket.Shutdown(SocketShutdown.Both);
				serverSocket.Close();
			} catch (Exception e) {
				serverSocket = null;
			}
		}
		IPHostEntry ipHostInfo = Dns.Resolve(serverAddress);
		IPAddress ipAddress = ipHostInfo.AddressList[0];
        IPEndPoint remoteEP = new IPEndPoint(ipAddress,serverPort);

        // Create a TCP/IP  socket.
        serverSocket = new Socket(AddressFamily.InterNetwork, 
                SocketType.Stream, ProtocolType.Tcp );


		SocketStateObject sso = new SocketStateObject(serverSocket, null);
		serverSocket.BeginConnect(remoteEP, new AsyncCallback(ProcessConnect), sso);
		
		setCurrentState(ConnectionState.Connecting);
		
	}

    
	private static bool sendPacketSync(string data)
    {
		tryConnect();
		if (serverSocket == null || ! serverSocket.Connected) {
			return false;
		}		

		byte [] toSend = Encoding.ASCII.GetBytes(data + "\n\n");

		try {
            int bytesSent = 0;
            int sent = 0;
            do
            {
                sent = serverSocket.Send(toSend);
                bytesSent += sent;

            } while (bytesSent < toSend.Length && sent > 0); 
		} catch (Exception e) {
			Debug.Log (e);
			return false;
		}
	
			return true;
        }

	private static string recvPacketSync()
    {
		tryConnect();
		if (serverSocket == null || ! serverSocket.Connected) {
			return "";
		}		
	
        byte[] buf = new byte[1024];
        string returnval = "";
	
		try {
            do
            {
                int bytesRec = serverSocket.Receive(buf);
                string temp = Encoding.ASCII.GetString(buf, 0, bytesRec);
                returnval += temp;
            } while (returnval.IndexOf("\n\n") < 0);
		} catch (Exception e) {
			Debug.Log (e);
			return "";
		} 
	
        return returnval.Substring(0, returnval.IndexOf("\n\n") + 1);
    }
	
	
	
	private static void sendPacket(string data)
    {
		if (serverSocket == null || ! serverSocket.Connected) {
			setCurrentState(ConnectionState.Disconnected);
			return;
		}
		
		SocketStateObject sso = new SocketStateObject(serverSocket, data + "\n\n");
		
		sso.socket.BeginSend(sso.buffer, 0, sso.buffer.Length, 0, new AsyncCallback(ProcessSend), sso);
//		Debug.Log ("Sending: " + data);
		setCurrentState(ConnectionState.Sending);
    }

	private static void recvPacket()
    {
		if (serverSocket == null || ! serverSocket.Connected) {
			setCurrentState(ConnectionState.Disconnected);
			return;
		}		
	
		SocketStateObject sso = new SocketStateObject(serverSocket, null);
		sso.socket.BeginReceive(sso.buffer, 0, sso.buffer.Length, 0, new AsyncCallback(ProcessReceive), sso);
		setCurrentState(ConnectionState.Receiving);

	}
	
	private class FalconDataStruct {
		public Vector3 tipPosition;
		public Vector3 godPosition;
		public Vector3 curforces;
		public bool[] buttons;
		
	}
	
	private class DynObjStruct {
		public Vector3 pos;
		public Quaternion orient;
		
	}
	
	private class SocketStateObject {
		
		public Socket socket;
		public byte[] buffer;
		public int sent = 0;
		public StringBuilder sb = new StringBuilder();
		
		public SocketStateObject(Socket s, string packet) {
			socket = s;
			if (packet != null)
				buffer = Encoding.ASCII.GetBytes(packet);
			else
				buffer = new byte[1024];
		}
	}
	
	static void ProcessConnect(IAsyncResult result) {
		try {
			Socket socket = ((SocketStateObject)result.AsyncState).socket;
				
			socket.EndConnect(result);
			
			setCurrentState(ConnectionState.Idle);
			
		} catch(Exception e) {
			setCurrentState(ConnectionState.Disconnected);
		}
	}
	
	static void ProcessSend(IAsyncResult result) {
//		Debug.Log ("In ProcessSend");

		try {
			SocketStateObject sso = (SocketStateObject)result.AsyncState;

			int bytesSent = serverSocket.EndSend(result);
			
			sso.sent += bytesSent;
//			Debug.Log (sso.sent.ToString() + " " + sso.buffer.Length.ToString());
			
			if (sso.sent < sso.buffer.Length) {
				sso.socket.BeginSend(sso.buffer,sso.sent, sso.buffer.Length - sso.sent, 0, new AsyncCallback(ProcessSend), sso);
			} else {
			
//				Debug.Log ("Why am I not Here?");
				if (packetQueue.Count > 0) {
					packetQueue.Dequeue();					
				}					
					
				packetCache = null;
				recvPacket ();
			}
			
		} catch (Exception e) {
			setCurrentState(ConnectionState.Disconnected);
		}
	}
	
	static void ProcessReceive(IAsyncResult result) {
		try {
//			Debug.Log ("In ProcessReceive");
			SocketStateObject sso = (SocketStateObject)result.AsyncState;

			int bytesRec = sso.socket.EndReceive(result);
			
			if (bytesRec > 0) {
				sso.sb.Append(Encoding.ASCII.GetString(sso.buffer,0,bytesRec));
//				Debug.Log ("Received So Far:");
	//			Debug.Log (sso.sb.ToString());
				if (sso.sb.ToString().IndexOf("\n\n") < 0) {
					// more to receive
					sso.buffer = new byte[sso.buffer.Length];
					sso.socket.BeginReceive(sso.buffer,0,sso.buffer.Length,0, new AsyncCallback(ProcessReceive), sso);
				} else {
					string tempPacket = sso.sb.ToString();
			        tempPacket = tempPacket.Substring(0, tempPacket.IndexOf("\n\n") + 1);
					
					if (tempPacket.Split('\n').Length <= updatePacketMinSize && packetQueue.Count > 0) {
						// Optimization
						// this isn't an update response so we don't care about it and there's more packets waiting to be sent.  Go ahead and start a new send.
						packetCache = null;
						sendPacket((string)packetQueue.Peek());
						
					} else {
						packetCache = tempPacket;
						
						setCurrentState(ConnectionState.Idle);
					}
				}
			} else {
				setCurrentState(ConnectionState.Disconnected);
			}
			
		} catch (Exception e) {
			setCurrentState(ConnectionState.Disconnected);
		}
	}
	
	static void socketThread() {
		
		while(true) {
			int waitTime = -1;
			if (getCurrentState() == ConnectionState.Disconnected) {
				waitTime = 100;
			} 
			if (packetQueue.Count == 0) {
				queueEvent.Reset();
				if (packetQueue.Count == 0) {
					queueEvent.WaitOne(waitTime);
				}
			}
			try {
				
				
				// are we disconnected? if so, try to connect;
				if (serverSocket == null || serverSocket.Connected == false || getCurrentState() == ConnectionState.Disconnected) {
					setCurrentState(ConnectionState.Connecting);
					if (serverSocket != null) {
				
						// we need to disconnect the current connection;
						try {
							serverSocket.Shutdown(SocketShutdown.Both);
							serverSocket.Close();
						} catch (Exception e) {
							serverSocket = null;
						}
					}
					IPHostEntry ipHostInfo = Dns.Resolve(serverAddress);
					IPAddress ipAddress = ipHostInfo.AddressList[0];
			        IPEndPoint remoteEP = new IPEndPoint(ipAddress,serverPort);
			
			        // Create a TCP/IP  socket.
			        serverSocket = new Socket(AddressFamily.InterNetwork, 
	                SocketType.Stream, ProtocolType.Tcp );
					
					
					serverSocket.Connect(remoteEP);
					
					setCurrentState(ConnectionState.Idle);
				}
				
				
				if (packetQueue.Count == 0)
					continue;
				
				setCurrentState(ConnectionState.Sending);
				
				string packet = (string) packetQueue.Peek();
				byte [] toSend = Encoding.ASCII.GetBytes(packet + "\n\n");

	            int bytesSent = 0;
	            int sent = 0;
	            do
	            {
	                sent = serverSocket.Send(toSend);
	                bytesSent += sent;
	
	            } while (bytesSent < toSend.Length && sent > 0); 

			
				packetQueue.Dequeue();
				
				setCurrentState(ConnectionState.Receiving);
				
		        byte[] buf = new byte[1024];
		        string returnval = "";
		
	            do
	            {
	                int bytesRec = serverSocket.Receive(buf);
	                string temp = Encoding.ASCII.GetString(buf, 0, bytesRec);
	                returnval += temp;
	            } while (returnval.IndexOf("\n\n") < 0);
				
				returnval = returnval.Substring(0, returnval.IndexOf("\n\n") + 1);
				
				inPacketQueue.Enqueue(returnval);
				
				if (packetQueue.Count == 0)
					setCurrentState(ConnectionState.Idle);

				
			} catch (Exception e) {
				setCurrentState(ConnectionState.Disconnected);
			}
			
		}
		
	}
}

#else
using System.Runtime.InteropServices;

public class FalconUnityCall{

	
	public static bool IsRemote() {
		return false;
	}

	private static int numfalcons = -1;
	
	public static int getNumFalcons() {
		return numfalcons;
	}
	
	
	// called ONCE to initialize falcons
	[DllImport("falconunity")]
	public static extern int initFalconUnity(); //returns number of falcons
	
	// called to stop falcons and clean up physics world
	[DllImport("falconunity")]
    public static extern bool closeFalconUnity(); //close all falcons
	
	// returns physics world frames per second
	[DllImport("falconunity")]
    public static extern float getFPS();
	
	// returns the position of the haptic tip
	[DllImport("falconunity")]
    public static extern bool getTipPosition(int falcon_num, float[] pos_out);
	
	// returns the position of the god object, note that this is a physics-simulated rigid body
	[DllImport("falconunity")]
    public static extern bool getGodPosition(int falcon_num, float[] pos_out);
	
	[DllImport("falconunity")]
    public static extern bool getFalconForces(int falcon_num, float[] force_out);
	

	// apply a static force to the haptic tip every tick
	[DllImport("falconunity")]
    public static extern bool setForceField(int falcon_num, float[] force); //set the falcon to send this constant force
	
	// apply a force to the god object for a specified amount of time
	[DllImport("falconunity")]
	public static extern bool applyForce(int falcon_num, float [] force, float time_in_secs);

	// set the gravity force applied to all objects
	[DllImport("falconunity")]
	public static extern bool setGravity(float [] force);
	
	// old, don't use
	[DllImport("falconunity")]
    public static extern void getLastError(StringBuilder buffer); //pass the error out
   
	// initialize god object to a sphere.  Calling this again will delete old god object and replace it with a new one
	[DllImport("falconunity")]
	public static extern bool setSphereGodObject(int falcon_num, float radius, float mass, float [] pos, float minDist, float maxDist); //set end effector to a sphere

	[DllImport("falconunity")]
	public static extern bool setRigidBodyGodObject(int falcon_num, int body_num, float minDist, float maxDist); 

	[DllImport("falconunity")]
	public static extern bool removeGodObject(int falcon_num); 


	// interpolate the pose given haptic tip over the given time period.  interpolation is done at each physics tick
	[DllImport("falconunity")]
	public static extern bool updateHapticTransform(int falcon_num, float [] pos, float [] rot, float [] scale, bool useCompensator, float time_in_secs); 

	// gets the states of the falcon's buttons
	[DllImport("falconunity")]
	public static extern bool getFalconButtonStates(int falcon_num,int[] buttons);
    
	// has the falcon been calibrated yet?
	[DllImport("falconunity")]
	public static extern bool getCalibrated(int falcon_num);
	
	// not implemented, don't use
	[DllImport("falconunity")]
	public static extern bool startLog(int falcon_num);
	
	// not implemented, don't use
	[DllImport("falconunity")]
	public static extern bool stopLog(int falcon_num, int num_entries, float [] god_obj_positions, float [] haptic_tip_positions, float [] god_obj_forces, float [] haptic_tip_forces, float [] delta_ts);
	
	// place a concave shape in the physics world with the following parameters 
	[DllImport("falconunity")]
	public static extern bool sendDynamicShape(int body_num, float [] shape, int num_tris, float weight, float k, float [] startPos, float []startOrient, float [] linearFactors, float [] angularFactors, float friction );
	
	// remove the given shape from the physics world
	[DllImport("falconunity")]
	public static extern bool removeDynamicShape(int body_num);
	
	// get the pose of the given shape
	[DllImport("falconunity")]
	public static extern bool getDynamicShapePose(int body_num, float [] pos, float [] orient);
	
	// set the pose of the given shape *** WARNING *** resets the shape's velocity
	[DllImport("falconunity")]
	public static extern bool setDynamicShapePose(int body_num, float [] pos, float [] orient);
	
	// apply force to the given shape
	[DllImport("falconunity")]
	public static extern bool applyForceToShape(int body_num, float [] linear, float [] torque);
	
	// update the properties of the given shape
	[DllImport("falconunity")]
	public static extern bool updateDynamicShape(int body_num, float weight, float hardness, float [] linearFactors, float [] angularFactors, float friction );
	
	
	// attach a 6 dimensional damped spring to an existing concave shape
	[DllImport("falconunity")]
	public static extern bool addSpringToShape(int body_num, int spring_num, float max_force, float dampingFactor, float [] goalPos, float [] goalOrient, float [] posConstraintLower, float [] posConstraintUpper, float [] orientConstraintLower, float [] orientConstraintUpper, int [] directionality);
	
	// remove the specified spring from its shape, does not remove the shape itself
	[DllImport("falconunity")]
	public static extern bool removeSpring(int spring_num);
	
	// move the pose of the spring and set parameters
	[DllImport("falconunity")]
	public static extern bool setSpring(int spring_num, float max_force, float dampingFactor, float []goalPos, float []goalOrient, float [] posConstraintLower, float [] posConstraintUpper, float [] orientConstraintLower, float [] orientConstraintUpper, int [] directionality);
	
	// interpolate the pose and parameters of the given spring over the given time period.  interpolation is done at each physics tick
	[DllImport("falconunity")]
	public static extern bool lerpSpring(int spring_num, float max_force, float dampingFactor, float []goalPos, float []goalOrient, float [] posConstraintLower, float [] posConstraintUpper, float [] orientConstraintLower, float [] orientConstraintUpper, int [] directionality, float time_in_secs);
	

	
	
	// Unity interface
	
	public static void setServerParams(string address, int port) {
	}
	
	// Start, Stop, Update, GetLastError are synchronous and immediate, all other methods place packets into a queue to be transmitted on next call to Update()
	
	// called ONCE to initialize falcons
	public static void Start() {
		numfalcons = initFalconUnity();

		
	} //returns number of falcons
	
	// called to stop falcons and clean up physics world
    public static bool Stop() {
		return closeFalconUnity();
	}
	
	public static void disconnect() {

	}
	
	// should be called once a frame
	public static bool Update() {
		return true;
	}
	

	// returns the position of the haptic tip
    public static bool getTipPosition(int falcon_num, out Vector3 pos_out) {
		
		float [] pos = new float[3];
		bool returnval = getTipPosition(falcon_num, pos);
		pos_out = new Vector3(pos[0], pos[1], pos[2]);
		
		return returnval;
	}
	
	// returns the position of the god object, note that this is a physics-simulated rigid body
    public static bool getGodPosition(int falcon_num, out Vector3 pos_out) {
		float [] pos = new float[3];
		bool returnval = getGodPosition(falcon_num, pos);
		pos_out = new Vector3(pos[0], pos[1], pos[2]);
		
		return returnval;
	}
		
	// returns the position of the god object, note that this is a physics-simulated rigid body
    public static bool getFalconForces(int falcon_num, out Vector3 force_out) {
		float [] force = new float[3];
		bool returnval = getFalconForces(falcon_num, force);
		force_out = new Vector3(force[0], force[1], force[2]);
		
		return returnval;
	}
	
	// apply a static force to the haptic tip every tick
    public static void setForceField(int falcon_num, Vector3 force) {
		
		float [] f = new float[3] {force.x, force.y, force.z};
		
		setForceField(falcon_num, f);
	}
	
	// apply a force to the god object for a specified amount of time
	public static void applyForce(int falcon_num, Vector3 force, float time_in_secs) {
		float [] f = new float[3] {force.x, force.y, force.z};
		
		applyForce(falcon_num, f, time_in_secs);
	}

	// set the gravity force applied to all objects
	public static void setGravity(Vector3 force) {
		float [] f = new float[3] {force.x, force.y, force.z};
		
		setGravity(f);
	}
	
    public static string getLastError() {
		StringBuilder buf = new StringBuilder();
		getLastError(buf);
		return buf.ToString();
	} 
   
	// initialize god object to a sphere.  Calling this again will delete old god object and replace it with a new one
	public static void setSphereGodObject(int falcon_num, float radius, float mass, Vector3 pos, float minDist, float maxDist) {
		float [] p = new float[3] {pos.x, pos.y, pos.z};
		
		setSphereGodObject(falcon_num, radius, mass, p, minDist, maxDist);
	}
    
	// gets the states of the falcon's buttons
	public static bool getFalconButtonStates(int falcon_num, out bool[] buttons) {
		int [] b = new int[4];
		
		bool returnval = getFalconButtonStates(falcon_num, b);
		
		buttons = new bool[4];
		
		for(int i=0;i<4;i++){
			if(b[i] == 0){
				buttons[i] = false;
			}
			else {
				buttons[i] = true;
			}
		}
		
		return returnval;
	}
		
	// place a concave shape in the physics world with the following parameters 
	public static void sendDynamicShape(int body_num, float [] shape, int num_tris, float weight, float k, Vector3 startPos, Quaternion startOrient, Vector3 linearFactors, Vector3 angularFactors, float friction ) {
		
		float [] pos = new float[3] {startPos.x, startPos.y, startPos.z};
		float [] orient = new float[4] {startOrient.x, startOrient.y, startOrient.z, startOrient.w};
		float [] lf = new float[3] {linearFactors.x, linearFactors.y, linearFactors.z};
		float [] af = new float[3] {angularFactors.x, angularFactors.y, angularFactors.z};
		
		
		sendDynamicShape(body_num, shape, num_tris, weight, k, pos, orient, lf, af, friction);
		
	}
	
	
	// get the pose of the given shape
	public static bool getDynamicShapePose(int body_num, out Vector3 pos, out Quaternion orient) {
		float [] p = new float[3];
		float [] o = new float[4];		
		
		
		bool returnval = getDynamicShapePose(body_num, p, o);
		
		pos = new Vector3(p[0], p[1], p[2]);
		orient = new Quaternion(o[0], o[1], o[2], o[3]);
		
		return returnval;
	}
	
	// set the pose of the given shape *** WARNING *** resets the shape's velocity
	public static void setDynamicShapePose(int body_num, Vector3 pos, Quaternion orient) {
		float [] p = new float[3] {pos.x, pos.y, pos.z};
		float [] o = new float[4] {orient.x, orient.y, orient.z, orient.w};
		
		setDynamicShapePose(body_num, p, o);
	}
	
	// apply force to the given shape
	public static void applyForceToShape(int body_num, Vector3 linear, Vector3 torque) {
		float [] l = new float[3] {linear.x, linear.y, linear.z};
		float [] t = new float[3] {torque.x, torque.y, torque.z};
		
		applyForceToShape(body_num, l, t);		
	}
	
	// update the properties of the given shape
	public static void updateDynamicShape(int body_num, float weight, float hardness, Vector3 linearFactors, Vector3 angularFactors, float friction ) {

		float [] lf = new float[3] {linearFactors.x, linearFactors.y, linearFactors.z};
		float [] af = new float[3] {angularFactors.x, angularFactors.y, angularFactors.z};
		
		
		updateDynamicShape(body_num, weight, hardness, lf, af, friction);
		
	}
	
	
	// attach a 6 dimensional damped spring to an existing concave shape
	public static void addSpringToShape(int body_num, int spring_num, float max_force, float dampingFactor, Vector3 goalPos, Quaternion goalOrient, Vector3 posConstraintLower, Vector3 posConstraintUpper, Vector3 orientConstraintLower, Vector3 orientConstraintUpper, int [] directionality) {

		float [] pos = new float[3] {goalPos.x, goalPos.y, goalPos.z};
		float [] orient = new float[4] {goalOrient.x, goalOrient.y, goalOrient.z, goalOrient.w};
		float [] pl = new float[3] {posConstraintLower.x, posConstraintLower.y, posConstraintLower.z};
		float [] pu = new float[3] {posConstraintUpper.x, posConstraintUpper.y, posConstraintUpper.z};
		float [] ol = new float[3] {orientConstraintLower.x, orientConstraintLower.y, orientConstraintLower.z};
		float [] ou = new float[3] {orientConstraintUpper.x, orientConstraintUpper.y, orientConstraintUpper.z};
		
		addSpringToShape(body_num, spring_num, max_force, dampingFactor, pos, orient, pl, pu, ol, ou, directionality);
	
	}

	
	// move the pose of the spring and set parameters
	public static void setSpring(int spring_num, float max_force, float dampingFactor, Vector3 goalPos, Quaternion goalOrient, Vector3 posConstraintLower, Vector3 posConstraintUpper, Vector3 orientConstraintLower, Vector3 orientConstraintUpper, int [] directionality) {
		float [] pos = new float[3] {goalPos.x, goalPos.y, goalPos.z};
		float [] orient = new float[4] {goalOrient.x, goalOrient.y, goalOrient.z, goalOrient.w};
		float [] pl = new float[3] {posConstraintLower.x, posConstraintLower.y, posConstraintLower.z};
		float [] pu = new float[3] {posConstraintUpper.x, posConstraintUpper.y, posConstraintUpper.z};
		float [] ol = new float[3] {orientConstraintLower.x, orientConstraintLower.y, orientConstraintLower.z};
		float [] ou = new float[3] {orientConstraintUpper.x, orientConstraintUpper.y, orientConstraintUpper.z};	
		
		setSpring (spring_num, max_force, dampingFactor, pos, orient, pl, pu, ol, ou, directionality);
	}
	
	// interpolate the pose and parameters of the given spring over the given time period.  interpolation is done at each physics tick
	public static void lerpSpring(int spring_num, float max_force, float dampingFactor, Vector3 goalPos, Quaternion goalOrient, Vector3 posConstraintLower, Vector3 posConstraintUpper, Vector3 orientConstraintLower, Vector3 orientConstraintUpper, int [] directionality, float time_in_secs) {
		float [] pos = new float[3] {goalPos.x, goalPos.y, goalPos.z};
		float [] orient = new float[4] {goalOrient.x, goalOrient.y, goalOrient.z, goalOrient.w};
		float [] pl = new float[3] {posConstraintLower.x, posConstraintLower.y, posConstraintLower.z};
		float [] pu = new float[3] {posConstraintUpper.x, posConstraintUpper.y, posConstraintUpper.z};
		float [] ol = new float[3] {orientConstraintLower.x, orientConstraintLower.y, orientConstraintLower.z};
		float [] ou = new float[3] {orientConstraintUpper.x, orientConstraintUpper.y, orientConstraintUpper.z};	
		
		lerpSpring (spring_num, max_force, dampingFactor, pos, orient, pl, pu, ol, ou, directionality, time_in_secs);
	}
	
	public static void updateHapticTransform(int falcon_num, Vector3 position, Quaternion rotation, Vector3 scale, bool useCompensator, float time_in_secs) {
		float [] pos = new float[3] {position.x, position.y, position.z};
		float [] orient = new float[4] {rotation.x, rotation.y, rotation.z, rotation.w};
		float [] s = new float[3] {scale.x, scale.y, scale.z};
		
		updateHapticTransform(falcon_num, pos, orient, s, useCompensator, time_in_secs);
	}

}
#endif



public class FalconUnity{
	
	
	public static bool IsRemote() {
		return FalconUnityCall.IsRemote();
	}

	
	public static void setServerParams(string address, int port) {
		FalconUnityCall.setServerParams(address, port);
	}
	
	// Start, Stop, Update, GetLastError are synchronous and immediate, all other methods place packets into a queue to be transmitted on next call to Update()
	
	// called ONCE to initialize falcons
	public static void Start() {
		FalconUnityCall.Start();
		
	} //returns number of falcons
	
	public static int getNumFalcons() {
		return FalconUnityCall.getNumFalcons();
	}
	
	// called to stop falcons and clean up physics world
    public static bool Stop() {
		return FalconUnityCall.Stop();
	}
	
	public static void disconnect() {
		FalconUnityCall.disconnect();

	}
	
	// should be called once a frame
	public static bool Update() {
		return FalconUnityCall.Update();
	}
	
	// returns physics world frames per second
    public static float getFPS() {
		return FalconUnityCall.getFPS();
	}
	
	// returns the position of the haptic tip
    public static bool getTipPosition(int falcon_num, out Vector3 pos_out) {
		return FalconUnityCall.getTipPosition(falcon_num, out pos_out);
	}
	
	// returns the position of the god object, note that this is a physics-simulated rigid body
    public static bool getGodPosition(int falcon_num, out Vector3 pos_out) {
		return FalconUnityCall.getGodPosition(falcon_num, out pos_out);
	}
	
	// returns the position of the god object, note that this is a physics-simulated rigid body
    public static bool getAverageFalconForces(int falcon_num, out Vector3 force_out) {
		return FalconUnityCall.getFalconForces(falcon_num, out force_out);
	}
	
	// apply a constant force to the haptic tip every tick
    public static void setForceField(int falcon_num, Vector3 force) {
		FalconUnityCall.setForceField(falcon_num, force);
	}
	
	// apply a force to the god object for a specified amount of time
	public static void applyForce(int falcon_num, Vector3 force, float time_in_secs) {
		FalconUnityCall.applyForce(falcon_num, force, time_in_secs);
	}

	// set the gravity force applied to all objects
	public static void setGravity(Vector3 force) {
		FalconUnityCall.setGravity(force);
	}
	
    public static string getLastError() {
		return FalconUnityCall.getLastError();
	} 
   
	// initialize god object to a sphere.  Calling this again will delete old god object and replace it with a new one
	public static void setSphereGodObject(int falcon_num, float radius, float mass, Vector3 pos, float minDistToMaxForce, float maxDistToMaxForce) {
		FalconUnityCall.setSphereGodObject(falcon_num, radius, mass, pos, minDistToMaxForce, maxDistToMaxForce);
	}

	// initialize god object to a sphere.  Calling this again will remove the old god object and replace it with a new one
	public static void setRigidBodyGodObject(int falcon_num, int body_num, float minDistToMaxForce, float maxDistToMaxForce) {
		FalconUnityCall.setRigidBodyGodObject(falcon_num, body_num, minDistToMaxForce, maxDistToMaxForce);
	}

	// detach any god object currently bound to the haptic tip (delete it in the case of a sphere)
	public static void removeGodObject(int falcon_num) {
		FalconUnityCall.removeGodObject(falcon_num);
	}
    
	// gets the states of the falcon's buttons
	public static bool getFalconButtonStates(int falcon_num, out bool[] buttons) {
		return FalconUnityCall.getFalconButtonStates(falcon_num, out buttons);
	}
		
	// place a concave shape in the physics world with the following parameters 
	public static void sendDynamicShape(int body_num, float [] shape, int num_tris, float weight, float k, Vector3 startPos, Quaternion startOrient, Vector3 linearFactors, Vector3 angularFactors, float friction ) {
		FalconUnityCall.sendDynamicShape(body_num, shape, num_tris, weight, k, startPos, startOrient, linearFactors, angularFactors, friction);
	}
	
	// remove the given shape from the physics world
	public static void removeDynamicShape(int body_num) {
		FalconUnityCall.removeDynamicShape(body_num);
	}
	
	// get the pose of the given shape
	public static bool getDynamicShapePose(int body_num, out Vector3 pos, out Quaternion orient) {
		return FalconUnityCall.getDynamicShapePose(body_num, out pos, out orient);
	}
	
	// set the pose of the given shape *** WARNING *** resets the shape's velocity
	public static void setDynamicShapePose(int body_num, Vector3 pos, Quaternion orient) {
		FalconUnityCall.setDynamicShapePose(body_num, pos, orient);
	}
	
	// apply force to the given shape
	public static void applyForceToShape(int body_num, Vector3 linear, Vector3 torque) {
		FalconUnityCall.applyForceToShape(body_num, linear, torque);
	}
	
	// update the properties of the given shape
	public static void updateDynamicShape(int body_num, float weight, float hardness, Vector3 linearFactors, Vector3 angularFactors, float friction ) {
		FalconUnityCall.updateDynamicShape(body_num, weight, hardness, linearFactors, angularFactors, friction);
	}
	
	
	// attach a 6 dimensional damped spring to an existing concave shape
	public static void addSpringToShape(int body_num, int spring_num, float max_force, float dampingFactor, Vector3 goalPos, Quaternion goalOrient, Vector3 posConstraintLower, Vector3 posConstraintUpper, Vector3 orientConstraintLower, Vector3 orientConstraintUpper, int [] directionality) {
		FalconUnityCall.addSpringToShape(body_num, spring_num, max_force, dampingFactor, goalPos, goalOrient, posConstraintLower, posConstraintUpper, orientConstraintLower, orientConstraintUpper, directionality);
	}
	
	// remove the specified spring from its shape, does not remove the shape itself
	public static void removeSpring(int spring_num) {
		FalconUnityCall.removeSpring(spring_num);
	}
	
	// move the pose of the spring and set parameters
	public static void setSpring(int spring_num, float max_force, float dampingFactor, Vector3 goalPos, Quaternion goalOrient, Vector3 posConstraintLower, Vector3 posConstraintUpper, Vector3 orientConstraintCCW, Vector3 orientConstraintCW, int [] directionality) {
		FalconUnityCall.setSpring(spring_num, max_force, dampingFactor, goalPos, goalOrient, -posConstraintLower, posConstraintUpper, Mathf.Deg2Rad * -orientConstraintCCW, Mathf.Deg2Rad * orientConstraintCW, directionality);
	}
	
	// interpolate the pose and parameters of the given spring over the given time period.  interpolation is done at each physics tick
	public static void lerpSpring(int spring_num, float max_force, float dampingFactor, Vector3 goalPos, Quaternion goalOrient, Vector3 posConstraintLower, Vector3 posConstraintUpper, Vector3 orientConstraintCCW, Vector3 orientConstraintCW, int [] directionality, float time_in_secs) {
		FalconUnityCall.lerpSpring(spring_num, max_force, dampingFactor, goalPos, goalOrient, -posConstraintLower, posConstraintUpper, Mathf.Deg2Rad * -orientConstraintCCW, Mathf.Deg2Rad * orientConstraintCW, directionality, time_in_secs);

	}
	
	public static void updateHapticTransform(int falcon_num, Vector3 pos, Quaternion rotation, Vector3 scale, bool useCompensator, float time_in_secs) {
		FalconUnityCall.updateHapticTransform(falcon_num, pos, rotation, scale, useCompensator, time_in_secs);
	}
}
