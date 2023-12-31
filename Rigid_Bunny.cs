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

    Vector3 init_v = new Vector3(5, 3, 0);   // velocity
    Vector3 init_w = new Vector3(1, 1, 5);	// angular velocity

    float mass;                                 // mass
    Matrix4x4 I_ref;                            // reference inertia

    float linear_decay = 0.999f;                // for velocity decay
    float angular_decay = 0.999f;

    float friction_mu_t = 0.4f;                 // friction coefficient
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

        Matrix4x4 rotate_mat = Matrix4x4.Rotate(transform.rotation);
        Vector3 collision_avg_point = new Vector3();  // avg point of collision
        Vector3 collision_avg_vi = new Vector3();  // avg speed of collision

        // 计算碰撞点和碰撞点速度
        int collision_count = 0;
        Matrix4x4 angular_speed_mat = Get_Cross_Matrix(m_w);  // 获得角速度矩阵
        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3 t = transform.TransformPoint(vertices[i]);  // 取顶点的世界坐标碰撞检测
            if (!TestCollision(t, P, N)) continue;
            Vector3 Rri = rotate_mat.MultiplyPoint(vertices[i]);
            Vector3 vi = m_v + angular_speed_mat.MultiplyPoint(Rri);  // 顶点速度
            if (Vector3.Dot(vi, N) > 0) continue;  // 顶点速度与法向量同向，不碰撞

            collision_avg_point += Rri;
            collision_avg_vi += vi;
            collision_count++;
        }
        if (collision_count == 0) return;
        collision_avg_point /= collision_count;
        collision_avg_vi /= collision_count;

        // 计算 vi_new
        Vector3 vi_n = Vector3.Dot(collision_avg_vi, N) * N;
        Vector3 vi_t = collision_avg_vi - vi_n;
        float a = Mathf.Max(1.0f - friction_mu_t * (1.0f + restitution) * vi_n.magnitude / vi_t.magnitude, 0.0f);
        Vector3 vi_n_new = -restitution * vi_n;
        Vector3 vi_t_new = a * vi_t;
        Vector3 vi_new = vi_n_new + vi_t_new;
        // 计算 K
        Matrix4x4 P_mat = Get_Cross_Matrix(collision_avg_point); // 构造P*矩阵
        Matrix4x4 K = SubMat(MulMat(Matrix4x4.identity, 1.0f / mass), P_mat * I_ref.inverse * P_mat);
        // 计算 impluse J
        Vector3 J = K.inverse * (vi_new - collision_avg_vi);
        m_v += J / mass;
        m_w += I_ref.inverse.MultiplyPoint(P_mat.MultiplyPoint(J));

        restitution *= 0.5f;
    }

    static Matrix4x4 MulMat(Matrix4x4 a, float b)
    {
        Matrix4x4 ret = new Matrix4x4();
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++) ret[i, j] = a[i, j] * b;
        return ret;
    }

    static Matrix4x4 SubMat(Matrix4x4 a, Matrix4x4 b)
    {
        Matrix4x4 ret = new Matrix4x4();
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++) ret[i, j] = a[i, j] - b[i, j];
        return ret;
    }

    Quaternion AddQuat(Quaternion s, Quaternion v)
    {
        Quaternion q = new Quaternion(s.x + v.x, s.y + v.y, s.z + v.z, s.w + v.w);
        return q;
    }

    void ResetEntityPos()
    {
        m_v = new Vector3();
        m_w = new Vector3();
        transform.position = new Vector3(0, 0.6f, 0);
        launched = false;
    }

    void InitEntitySpeed()
    {
        m_v = init_v;
        m_w = init_w;
        friction_mu_t = 0.4f;
        restitution = 0.5f;
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

        Quaternion dq = new Quaternion(dt * m_w.x / 2, dt * m_w.y / 2, dt * m_w.z / 2, 0.0f);
        q = AddQuat(q, dq * q);
        q = Quaternion.Normalize(q);

        // Part IV: Assign to the object
        transform.position = x;
        transform.rotation = q;
    }
}