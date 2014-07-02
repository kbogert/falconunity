using UnityEngine;
using System.Collections;

public class SimpleDebug : MonoBehaviour {

	// Use this for initialization
	void Start () {
	
	}
	
	// Update is called once per frame
	void Update () {
	
	}
	
	void OnGUI() {
		float val = FalconUnity.getFPS();
		
		
		GUI.Label(new Rect(5,5,200,30), val.ToString()  );
	}
}
