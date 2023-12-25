using UnityEngine;
using System.Collections;
using System.Diagnostics;

public class Rigid_Bunny : MonoBehaviour
{
    bool launched = false;
    bool is_first_time = false;
    float dt = 0.015f;
    Vector3 v = new Vector3(0, 0, 0);   // velocity
    Vector3 w = new Vector3(0, 0, 0);	// angular velocity

    Vector3 temp_update_pos = new Vector3(0, 0, 0);
    Vector3 gravity = new Vector3(0, -9.8f, 0);

    float mass;                                 // mass
    Matrix4x4 I_ref;                            // reference inertia

    float linear_decay = 0.999f;                // for velocity decay
    float angular_decay = 0.98f;
    float restitution = 0.5f;                   // for collision


    // Use this for initialization
    void Start()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;

        float m = 1;
        mass = 0;
        for (int i = 0; i < vertices.Length; i++)
        {
            mass += m;
            float diag = m * vertices[i].sqrMagnitude;
            I_ref[0, 0] += diag;
            I_ref[1, 1] += diag;
            I_ref[2, 2] += diag;
            I_ref[0, 0] -= m * vertices[i][0] * vertices[i][0];
            I_ref[0, 1] -= m * vertices[i][0] * vertices[i][1];
            I_ref[0, 2] -= m * vertices[i][0] * vertices[i][2];
            I_ref[1, 0] -= m * vertices[i][1] * vertices[i][0];
            I_ref[1, 1] -= m * vertices[i][1] * vertices[i][1];
            I_ref[1, 2] -= m * vertices[i][1] * vertices[i][2];
            I_ref[2, 0] -= m * vertices[i][2] * vertices[i][0];
            I_ref[2, 1] -= m * vertices[i][2] * vertices[i][1];
            I_ref[2, 2] -= m * vertices[i][2] * vertices[i][2];
        }
        I_ref[3, 3] = 1;
    }

    Matrix4x4 Get_Cross_Matrix(Vector3 a)
    {
        //Get the cross product matrix of vector a
        Matrix4x4 A = Matrix4x4.zero;
        A[0, 0] = 0;
        A[0, 1] = -a[2];
        A[0, 2] = a[1];
        A[1, 0] = a[2];
        A[1, 1] = 0;
        A[1, 2] = -a[0];
        A[2, 0] = -a[1];
        A[2, 1] = a[0];
        A[2, 2] = 0;
        A[3, 3] = 1;
        return A;
    }

    // In this function, update v and w by the impulse due to the collision with
    //a plane <P, N>
    void Collision_Impulse(Vector3 P, Vector3 N)
    {
    }

    Quaternion multiplyForQuaternion(Vector4 multiplier, Quaternion q)
    {
        Vector3 v1 = new Vector3(multiplier.y, multiplier.z, multiplier.w);
        Vector3 v2 = new Vector3(q.y, q.z, q.w);
        Vector3 v3 = multiplier.x * v2 + q.x * v1 + Vector3.Cross(v1, v2);
        Quaternion ret = new Quaternion();
        ret.x = multiplier.x * q.x - Vector3.Dot(v1, v2); ;
        ret.y = v3.x;
        ret.z = v3.y;
        ret.w = v3.z;
        return ret;
    }

    void printQuaternion(Quaternion q)
    {
        Debug.Log("Quaternion q" + q.x + "," + q.y + "," + q.z + "," + q.w);
    }

    // Update is called once per frame
    void Update()
    {
        //Game Control
        if (Input.GetKey("r"))
        {
            v = new Vector3();
            w = new Vector3();
            transform.position = new Vector3(0, 0.6f, 0);
            restitution = 0.5f;
            launched = false;
        }
        if (Input.GetKey("l") && !launched)
        {
            v = new Vector3(5, 2, 0);
            w = new Vector3(5f, 5f, 0);
            launched = true;
        }

        // Part I: Update velocities
        float delta_time = Time.deltaTime;
        if (launched)
        {
            float first_time_param = 1.0f;
            if (is_first_time) first_time_param = .5f;  // mid-point
            is_first_time = false;
            // Velocity update
            v += gravity * delta_time * first_time_param;
        }

        // Part II: Collision Impulse
        Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
        Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

        // Part III: Update position & orientation
        //Update linear status
        Vector3 x = transform.position;
        v -= linear_decay * delta_time * v;
        x += v * delta_time;

        //Update angular status
        Quaternion q = transform.rotation;
        w -= angular_decay * delta_time * w;
        float half_dt = delta_time / 2.0f;
        Vector4 multiplier = new Vector4(0, half_dt * w.x, half_dt * w.y, half_dt * w.z);
        Quaternion add_q = multiplyForQuaternion(multiplier, q);
        q.x = q.x + add_q.x; q.y = q.y + add_q.y;
        q.z = q.z + add_q.z; q.w = q.w + add_q.w;

        // Part IV: Assign to the object
        transform.position = x;
        transform.rotation = q;
    }
}
