using System.Runtime.CompilerServices;
using UnityEngine;

public class Rigid_Bunny : MonoBehaviour
{
    bool launched = false;
    bool is_first_time = false;
    float dt = 0.015f;

    Vector3 m_v = new Vector3(0, 0, 0);   // velocity
    Vector3 m_w = new Vector3(0, 0, 0);	// angular velocity
    Vector3 gravity = new Vector3(0, -9.8f, 0);

    float mass;                                 // mass
    Matrix4x4 I_ref;                            // reference inertia

    float linear_decay = 0.999f;                // for velocity decay
    float angular_decay = 0.98f;
    float restitution = 0.5f;                   // for collision
    
    float friction_mu_t = 0.5f;                 // friction coefficient
    float friction_mu_n = 0.7f;


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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    bool TestCollision(Vector3 t, Vector3 P, Vector3 N)
    {
        return Vector3.Dot(t - P, N) < 0;
    }

    // In this function, update m_v and m_w by the impulse due to the collision with
    //a plane <P, N>
    void Collision_Impulse(Vector3 P, Vector3 N)
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;

        // rotate matrix, convert rotation to matrix
        Matrix4x4 rotate_mat = Matrix4x4.Rotate(transform.rotation);

        for (int i = 0; i < vertices.Length; i++)
        {
            // 获取顶点的当前世界坐标
            Vector3 t = transform.TransformPoint(vertices[i]);
            // 碰撞检测
            if (!TestCollision(t, P, N)) continue;
            // 计算 vi_new
            Vector3 p = t - P;
            Vector3 vi_n = Vector3.Dot(m_v, N) * N;
            Vector3 vi_t = m_v - vi_n;
            float a = Mathf.Max(1.0f - friction_mu_t * (1.0f + friction_mu_n) * vi_n.magnitude / vi_t.magnitude, 0.0f);
            Vector3 vi_n_new = -friction_mu_n * vi_n;
            Vector3 vi_t_new = a * vi_t;
            Vector3 vi_new = vi_n_new + vi_t_new;
            
            Vector3 p_new = rotate_mat * p;  // 计算旋转后的p
            Matrix4x4 P_mat = Get_Cross_Matrix(p_new); // 构造P*矩阵
            Matrix4x4 I4 = Matrix4x4.identity; // 获得单位阵
            // 计算K
            Matrix4x4 K = SubMate4x4(MulMat4x4(I4, 1.0f / mass), (P_mat * I_ref.inverse * P_mat));
            // 计算 impluse J
            Vector3 J = K.inverse * (vi_new - m_v);
            // 计算 v_new
            m_v += J / mass;
            // 计算 w_new
            Vector4 vector4 = I_ref.inverse * P_mat * J;  // 前3项为w
            m_w += new Vector3(vector4.x, vector4.y, vector4.z);
        }
    }

    static Matrix4x4 MulMat4x4(Matrix4x4 a, float b)
    {
        Matrix4x4 ret = new Matrix4x4();
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++) ret[i, j] = a[i, j] * b;
        return ret;
    }

    static Matrix4x4 SubMate4x4(Matrix4x4 a, Matrix4x4 b)
    {
        Matrix4x4 ret = new Matrix4x4();
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++) ret[i, j] = a[i, j] - b[i, j];
        return ret;
    }

    Quaternion MultiplyForQuaternion(Vector4 multiplier, Quaternion q)
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

    void ResetEntityPos()
    {
        m_v = new Vector3();
        m_w = new Vector3();
        transform.position = new Vector3(0, 0.6f, 0);
        restitution = 0.5f;
        launched = false;
    }

    void InitEntitySpeed()
    {
        m_v = new Vector3(5, 2, 0);
        m_w = new Vector3(5f, 5f, 0);
        launched = true;
    }

    // Update is called once per frame
    void Update()
    {
        //Game Control
        if (Input.GetKey("r"))
        {
            ResetEntityPos();
        }
        if (Input.GetKey("l") && !launched)
        {
            InitEntitySpeed();
        }

        // Part I: Update velocities
        float delta_time = Time.deltaTime;
        if (launched)
        {
            float first_time_param = 1.0f;
            if (is_first_time) first_time_param = .5f;  // mid-point
            is_first_time = false;
            // Velocity update
            m_v += gravity * delta_time * first_time_param;
        }

        // Part II: Collision Impulse
        Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
        Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

        // Part III: Update position & orientation
        //Update linear status
        Vector3 x = transform.position;
        m_v -= linear_decay * delta_time * m_v;
        x += m_v * delta_time;

        //Update angular status
        Quaternion q = transform.rotation;
        m_w -= angular_decay * delta_time * m_w;
        float half_dt = delta_time / 2.0f;
        Vector4 multiplier = new Vector4(0, half_dt * m_w.x, half_dt * m_w.y, half_dt * m_w.z);
        Quaternion add_q = MultiplyForQuaternion(multiplier, q);
        q.x += add_q.x; q.y += add_q.y;
        q.z += add_q.z; q.w += add_q.w;

        // Part IV: Assign to the object
        transform.position = x;
        transform.rotation = q;
    }
}
