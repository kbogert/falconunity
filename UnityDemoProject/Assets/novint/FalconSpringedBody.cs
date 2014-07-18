using UnityEngine;
using System.Collections;

public class FalconSpringedBody : MonoBehaviour {

	public float k = 0.5f;
 	public float mass = 1;
	public Vector3 device_scale = new Vector3(1.0f,1.0f,1.0f);//send 1/4 size in the z dimension
	public Vector3 linearFactors = new Vector3(1.0f,1.0f,1.0f);
	public Vector3 angularFactors = new Vector3(1.0f,1.0f,1.0f);
	public Vector3 springPos;
	public Quaternion springOrientation = new Quaternion(0,0,0,1);
	public Vector3 posConstraintLower = new Vector3(0,0,0);
	public Vector3 posConstraintUpper = new Vector3(0,0,0);
	public Vector3 orientConstraintCCW = new Vector3(0,0,0);
	public Vector3 orientConstraintCW = new Vector3(0,0,0);
	public float max_force = 1;
	public float dampingFactor = 1.0f;
	public float friction = 0.8f;
	
	public int [] directionality = new int[6] {0,0,0,0,0,0};
	
	public int bodyId = 0;
	public int springId = 0;
	
	// Use this for initialization
	void Start () {
		bodyId = FalconRigidBody.getNextBodyId();
		springId = getNextSpringId();
	 	refreshShape();
	 
	}
	
	public void refreshShape(){
		Mesh m = GetComponent<MeshFilter>().mesh;
   	 	Vector3[] v = m.vertices;
	 	int[] f = m.triangles;
	 	float[] shape = new float[f.Length*3];

		for(int i=0;i<f.Length;i++){
			Vector3 vert = v[f[i]];
				vert.Scale(transform.localScale);
				vert.Scale(device_scale);
//				vert = transform.localRotation*vert;
			
			shape[i*3] = vert.x;
			shape[i*3+1] = vert.y;
			shape[i*3+2] = vert.z;
		}
		Vector3 localPosition = transform.localPosition;
		localPosition.Scale(device_scale);

		FalconUnity.sendDynamicShape(bodyId,shape, f.Length/3, mass, k, localPosition, transform.localRotation, linearFactors, angularFactors, friction);
		FalconUnity.addSpringToShape(bodyId, springId, max_force, dampingFactor, springPos,springOrientation,posConstraintLower,posConstraintUpper, orientConstraintCCW, orientConstraintCW, directionality);
	}

	
	// Update is called once per frame
	public void FixedUpdate () {
		Vector3 pos;
		Quaternion orient;
		
		
		bool res = FalconUnity.getDynamicShapePose(bodyId, out pos, out orient);
		if(!res){
//			Debug.Log("Error getting object pose");
			return;
		}	
		transform.localPosition = pos;
		transform.localRotation = orient;

		FalconUnity.updateDynamicShape(bodyId, mass, k, linearFactors, angularFactors, friction);

		FalconUnity.lerpSpring(springId,max_force, dampingFactor, springPos,springOrientation,posConstraintLower,posConstraintUpper, orientConstraintCCW, orientConstraintCW, directionality, 0);
//		FalconUnity.setSpring(springId,max_force,goalPos,goalOrient,pConstraintl,pConstraintu, oConstraintl, oConstraintu);

	}

	static object Lock = new object();
	private static int curId = -1;
	public static int getNextSpringId() {
		lock(Lock) {
			curId ++;
			return curId;
		}
	}
}
