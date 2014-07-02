using UnityEngine;
using System.Collections;

public class Rotate : MonoBehaviour {
	
	public float speed;
	public Quaternion curRotation;
	void printAngleAxis(Quaternion q) {
		float a = 2 * Mathf.Acos(q.w);
		float x = q.x / Mathf.Sin(a / 2);
		float y = q.y / Mathf.Sin(a / 2);
		float z = q.z / Mathf.Sin(a / 2);
		Debug.Log (a);
		Debug.Log (x);
		Debug.Log (y);
		Debug.Log (z);
		
	}
	
	// Use this for initialization
	void Start () {
		curRotation = transform.rotation;
		Debug.Log (new Quaternion(0.869851f, 0, 0, -0.493315f).eulerAngles);
		Debug.Log (new Quaternion(-0.869851f, 0, 0, 0.493315f).eulerAngles);
		float angle;
		Vector3 axis;
		new Quaternion(0.869851f, 0, 0, -0.493315f).ToAngleAxis(out angle, out axis);
		Debug.Log (angle);
		Debug.Log (axis);
		printAngleAxis(new Quaternion(0.869851f, 0, 0, -0.493315f));
		new Quaternion(-0.869851f, 0, 0, 0.493315f).ToAngleAxis(out angle, out axis);
		Debug.Log (angle);
		Debug.Log (axis);
		printAngleAxis(new Quaternion(-0.869851f, 0, 0, 0.493315f));
	}
	
	// Update is called once per frame
	void Update () {
	
		Quaternion rotateAmount = Quaternion.AngleAxis(speed * Time.deltaTime, new Vector3(1,0,0));
		
		curRotation *= rotateAmount;
		
		FalconSpringedBody sb = GetComponent<FalconSpringedBody>();
		
//		FalconUnity.lerpSpring(sb.springId, sb.max_force, goalPos, orient, sb.
		sb.springOrientation = curRotation;
	}
}
