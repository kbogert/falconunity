using UnityEngine;
using System.Collections;
using System.Text;
using System.Net.Sockets;

public class FalconMain : MonoBehaviour {
	public int num_falcons;
	private int my_num;
	private static int my_num_falcons;
	public Vector3 gravity;
	
	public string address;
	public int port;
    public AddressFamily protocol;

	// Use this for initialization
	void Awake () {
		my_num = getNextNum();
		
		if (my_num == 0) {
			Debug.Log(address + " " + port + " " + protocol);
			FalconUnity.setServerParams(address, port, protocol);
			FalconUnity.Start();
			
			FalconUnity.Update();
			if (FalconUnity.IsRemote()) {
				FalconUnity.Update();
			}
			
			FalconUnity.setGravity(gravity);

		}
	}
	
	void OnApplicationQuit(){
		if (my_num == 0) {
			FalconUnity.Stop();
			FalconUnity.disconnect();
		}
	}
	
	// Update is called once per frame
	void Update () {
		if (my_num == 0) {
			num_falcons = FalconUnity.getNumFalcons();
			FalconUnity.Update();
		}
	}
	
	void Start () {
//		FalconUnity.setForceField(0, new float[] {0,-10,0});
		if (my_num == 0) {
			num_falcons = FalconUnity.getNumFalcons();
			if (FalconUnity.IsRemote()) {
				FalconUnity.Update();
			}
		}
	}
	
	
	static object Lock = new object();
	private static int curId = -1;
	public static int getNextNum() {
		lock(Lock) {
			curId ++;
			return curId;
		}
	}
}