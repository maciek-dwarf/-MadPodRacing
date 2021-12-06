
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <limits>


using namespace std;

namespace {
            
    constexpr int MAX_THRUST = 100;                
    constexpr float CHECKPOINT_RADIUS = 600;
    constexpr float POD_RADIUS = 400;
    
    constexpr float PI = 3.14159265359f;    
    constexpr int SIMULATION_LEVEL = 3;
    constexpr int OPPONENT_SIMULATION_LEVEL = 3;
    constexpr float SPEED_TO_SHIELD = 120.0f;
    constexpr float DOT_TO_SHIELD = 0.8f;
    
    constexpr float FORCE_THRUST_TRESHOLD = 2000.0f;
    constexpr float BIG_NUMBER = 30000.0f;
    constexpr float VERY_SMALL_NUMBER = -1000000000.0f;
    constexpr float INTERCEPTOR_SEARCH_TRESHOLD = 4500.0f;

}


class Vector2D
{
public:
    Vector2D(int xx, int yy)
    {
        x = xx;
        y = yy;
    }
    Vector2D(float xx = 0, float yy = 0)
    {
        x = xx;
        y = yy;
    }
    ~Vector2D() = default;

    void Rotate(const float angle)
    {
        float xt = (x * cosf(angle)) - (y * sinf(angle));
        float yt = (y * cosf(angle)) + (x * sinf(angle));
        x = xt;
        y = yt;
    };
    float Magnitude() const
    {
        return sqrtf(x * x + y * y);
    };
    float getAngle() const
    {
        float mag = Magnitude();
        float angle = acos(x / mag);
        if (y < 0.0f)
        {
            angle = 2.0f * PI - angle;
        }
        return angle;
    }

    float Normalize()
    {
        float mag = Magnitude();

        if (mag != 0.0f)
        {
            x /= mag;
            y /= mag;
        }

        return mag;

    };

    float DotProduct(const Vector2D& v2) const
    {
        return (x * v2.x) + (y * v2.y);
    };

    float CrossProduct(const Vector2D& v2) const
    {
        return (x * v2.y) - (y * v2.x);
    };

    static Vector2D Zero()
    {
        return Vector2D(0, 0);
    };

    static float Distance(const Vector2D& v1, const Vector2D& v2)
    {
        return sqrtf(pow((v2.x - v1.x), 2) + pow((v2.y - v1.y), 2));
    };

    Vector2D& operator= (const Vector2D& v2)
    {
        if (this == &v2)
            return *this;

        x = v2.x;
        y = v2.y;

        return *this;
    };

    Vector2D& operator+= (const Vector2D& v2)
    {
        x += v2.x;
        y += v2.y;

        return *this;
    };

    Vector2D& operator-= (const Vector2D& v2)
    {

        x -= v2.x;
        y -= v2.y;

        return *this;
    };

    Vector2D& operator*= (const float scalar)
    {
        x *= scalar;
        y *= scalar;

        return *this;
    };

    Vector2D& operator/= (const float scalar)
    {
        x /= scalar;
        y /= scalar;

        return *this;
    };

    const Vector2D operator+(const Vector2D& v2) const
    {
        return Vector2D(*this) += v2;
    };

    const Vector2D operator-(const Vector2D& v2) const
    {
        return Vector2D(*this) -= v2;
    };

    const Vector2D operator*(const float scalar) const
    {
        return Vector2D(*this) *= scalar;
    };

    const Vector2D operator/(const float scalar) const
    {
        return Vector2D(*this) /= scalar;
    };

    bool operator== (const Vector2D& v2) const
    {
        return ((x == v2.x) && (y == v2.y));
    };

    bool operator!= (const Vector2D& v2) const
    {
        return !((x == v2.x) && (y == v2.y));
    };

public:
    float x, y;
};


class PodData
{

public:
    PodData(const Vector2D& pos, const Vector2D& s, const int id, const float a) :
        position(pos), speed(s), check_point_id(id), angle(a), lap(0),
        passed_checkpoint(false), m(1.0f), score(0)
    {
    }

    PodData() = default;
    PodData(const PodData&) = default;

    //m mass 1 normal 10 shielded
    void UpdatePod(const Vector2D& pos, const Vector2D& s, const int a, const int ch_point_id)
    {
        position = pos;
        speed = s;
        if (check_point_id != ch_point_id)
        {
            score++;
        }
        if (check_point_id != ch_point_id && ch_point_id == 0)
        {
            lap++;
        }
        check_point_id = ch_point_id;
        angle = a;
        m = 1.0f;

    }

    
    Vector2D getDirection() const
    {
        return Vector2D(cos(angle * PI / 180.0f), sin(angle * PI / 180.0f));
    }

    //update angle
    void addAngle(float angleDiff) {
        angle += angleDiff;

        if (angle < 0.0f) {
            angle = 360.0f + angle;
        }
        else if (angle > 360.0f) {
            angle = angle - 360.0f;
        }
    }
    
    //solve quadric equation to check collision
    //needs some cleaning
    bool Collide(const PodData& p, float& t) const
    {
        t = numeric_limits<float>::max();
        Vector2D soffset = p.speed - speed;
        Vector2D poffset = p.position - position;
        float a = soffset.DotProduct(soffset);
        float b = soffset.DotProduct(poffset) * 2.0f;
        float c = poffset.DotProduct(poffset) - 4.0f * (float)(POD_RADIUS * POD_RADIUS * 1.1f);
        float rc = b * b - 4 * a * c;
        if (rc < 0 || a == 0.0f)
        {
            return false;
        }
        rc = sqrt(rc);
        float x1 = (-b - rc) / (2.0f * a);
        float x2 = (-b + rc) / (2.0f * a);
        if (x1 >= 0.0f && x1 <= 1.3f)
        {
            t = x1;
            return true;
        }
        if (x2 >= 0.0f && x2 <= 1.0f)
        {
            t = x2;
            return true;
        }

    
        return false;
    }

    //used to check collision with checkpoint
    bool Collide(const Vector2D& pposition, const Vector2D& pspeed, float& t) const
    {
        Vector2D soffset = speed;
        Vector2D poffset = pposition - position;
        if (poffset.Magnitude() < (float)(CHECKPOINT_RADIUS))
        {
            return true;
        }
        float dot = soffset.DotProduct(poffset) / soffset.Magnitude();

        if (dot < 0.0f)
        {
            return false;
        }
        if (poffset.DotProduct(poffset) - dot * dot > (float)(CHECKPOINT_RADIUS * CHECKPOINT_RADIUS))
        {
            return false;
        }
        return soffset.Magnitude() > dot;        
        
    }

    //simulation of collision
    //of 2 pods, needs more debugging
    // impulse is quite shady
    void CollisionSimulation(const PodData& pod, Vector2D& speed1, Vector2D& speed2) const
    {
        //direction 
        Vector2D u = (pod.position - position);
        //i assume that pod's mass is shielded   - 10.0f      
        float mm = (m * 10.0f) / (m + 10.0f);
        float fact = (pod.speed - speed).DotProduct(u);
        float impulse = -2 * mm * fact;
        if (impulse < 120.0f)
        {
            impulse = 120.0f;
        }
        
        speed1 = speed;
        speed1 += u * (1 / m) * impulse;
        speed2 = pod.speed;
        speed2 += u * (-1 / 10.0f) * impulse;
    }

    void moveWithCollision(const float t, const float thrust,
        PodData& cpod)
    {       

        Vector2D dir = getDirection();
        Vector2D cdir = cpod.getDirection();


        //we assume that with m =10 (shielded) speed is not updated
        //we assume that colided pod has mas 10 is shielded
        //cpod.speed += cdir * 100.0f;
        if (m < 9.0f) {
            speed += dir * (float)thrust;
        }

        position += speed * t;
        cpod.position += cpod.speed * t;

        Vector2D speed1, speed2;
        CollisionSimulation(cpod, speed1, speed2);
        speed = speed1;
        cpod.speed = speed2;

        position += speed * (1.0f - t);
        cpod.position += cpod.speed * (1.0f - t);

        int px = (int)position.x;
        int py = (int)position.y;

        position = Vector2D(px, py);

        px = (int)cpod.position.x;
        py = (int)cpod.position.y;

        cpod.position = Vector2D(px, py);


        //friction
        speed *= 0.85f;
        cpod.speed *= 0.85f;


        int sx = (int)speed.x;
        int sy = (int)speed.y;
        speed = Vector2D(sx, sy);

        sx = (int)cpod.speed.x;
        sy = (int)cpod.speed.y;
        cpod.speed = Vector2D(sx, sy);

    }


    //used for simulation - moves the pod
    //following expert's rules
    float move(float thrust)
    {
        Vector2D prevpos(position);

        float rangle = angle * (PI / 180.0f);
        Vector2D dir = getDirection();

        //if shielded we ignore thrust
        if (m < 9.0f) {
            speed += dir * (float)thrust;
        }       

        position += speed;
        int px = (int)position.x;
        int py = (int)position.y;
        position = Vector2D(px, py);

        float mag = (position - prevpos).Magnitude();
        //friction
        speed *= 0.85f;


        int sx = (int)speed.x;
        int sy = (int)speed.y;
        speed = Vector2D(sx, sy);
        return mag;

    }

    bool shouldShield(const PodData& pod) const
    {
        float mspeed = speed.Magnitude();
        float t;
        bool collide = Collide(pod, t);
       
        if (collide && mspeed > SPEED_TO_SHIELD)
        {
            return true;
        }
        return false;
    }



    bool shouldShield(const Vector2D& checkpoint_dir, const PodData& pod) const
    {
        float mspeed = speed.Magnitude();
        float t;
        bool collide = Collide(pod, t);
        Vector2D opp_pod_dir = pod.getDirection();       

        if (collide && mspeed > SPEED_TO_SHIELD && (opp_pod_dir.DotProduct(checkpoint_dir))
            < DOT_TO_SHIELD)
        {
            return true;
        }
        return false;
    }

    //number of checkpoints passed with weight    
    float getScore() const
    {
        return (float)score * BIG_NUMBER;
    }

    float getDistanceEval(const Vector2D& target,
        const bool isInterceptor, const PodData& opp) const
    {
        Vector2D offset = target - position;
        float dist = offset.Magnitude();
        offset.Normalize();
               
        float angle1 = offset.getAngle();
        
        float diff_angle = angle * PI / 180.0f - angle1;

        //return dist + pod.speed.Magnitude() * angle * angle * 10.0f;
        if (isInterceptor)
        {
            /* Vector2D offset1 = opp.position - position;
             float dot = opp.speed.DotProduct(speed);
             bool well_oriented  =  speed.DotProduct( offset1 ) > 0.0f;
             if(well_oriented && dot < -0.9f  && offset1.Magnitude() < 4000.0f)
             {
                 dist-= 2000.0f;
                 //return pods[opp_id].position;
             }*/
        }
        //cerr<< " dist " << dist<< endl;
        if (!isInterceptor && passed_checkpoint)
        {
            // better evaluation of distance after passing checkpoint
            // if we should slowdawn if we are badly oriented
            dist += speed.Magnitude() * abs(diff_angle) * 3.0f; 
        }
        return dist; 


    }
    

public:
    Vector2D position;
    Vector2D speed;
    int check_point_id;
    float angle;
    int lap;
    bool passed_checkpoint;
    float m; // weight of the object
    int score; // nr of total checkpoints passed

};

struct OutputValues {
    int out_x;
    int out_y;
    string sthrust;
};


class Game
{
private:

    vector<PodData> pods;
    vector<Vector2D> checkpoints;
    vector<bool> should_slowdawn;
    int laps;
    int nr_check_points;
    bool has_boost;

public:
    Game(const vector<Vector2D>& chpoints, int l) :
        checkpoints(chpoints), laps(l), has_boost(true)
    {
        pods.resize(4);
        nr_check_points = checkpoints.size();
    }
    Game() = default;

    void UpdatePod(const Vector2D& pos, const Vector2D& s, const int a, const int check_point_id, const int id)
    {
        pods[id].UpdatePod(pos, s, a, check_point_id);
    }
    void ComputeValues(OutputValues& out_pod1, OutputValues& out_pod2, bool& first)
    {
        int thrust1 = MAX_THRUST, thrusts2 = MAX_THRUST;

        int chaser = 0;
        int interceptor = 1;
        const Vector2D& checkpoint_pos = checkpoints[pods[chaser].check_point_id];
        Vector2D offset = checkpoint_pos - pods[chaser].position;


        float checkpoint_dist = offset.Magnitude();

        int opp_id = getWinningOpponentsID();
        if (!first)
        {

            PickThrustAngle(out_pod1, SIMULATION_LEVEL);
            Vector2D target = setTargetForInterceptor(pods[interceptor], opp_id);

            PickThrustAngleInterceptor(out_pod2, target, OPPONENT_SIMULATION_LEVEL);
            cerr << " pods[1].check_point_id " << pods[interceptor].check_point_id << endl;

        }
        else {
            first = false;
            out_pod1.out_x = checkpoint_pos.x;
            out_pod1.out_y = checkpoint_pos.y;
            out_pod1.sthrust = "BOOST";
            out_pod2.out_x = checkpoint_pos.x;
            out_pod2.out_y = checkpoint_pos.y;
            out_pod2.sthrust = "100";
        }
        Vector2D checkpoint_dir = checkpoints[pods[chaser].check_point_id] - pods[chaser].position;
        checkpoint_dir.Normalize();

        //finally we use simple check to determine shield
        //we check relative pos of 2 pods 
        if (pods[chaser].shouldShield(checkpoint_dir, pods[interceptor]))
        {
            out_pod1.sthrust = "SHIELD";
        }
        if (pods[chaser].shouldShield(checkpoint_dir, pods[2]))
        {
            out_pod1.sthrust = "SHIELD";
        }
        if (pods[chaser].shouldShield(checkpoint_dir, pods[3]))
        {
            out_pod1.sthrust = "SHIELD";
        }

        //shielding for interceptor
        checkpoint_dir = checkpoints[pods[opp_id].check_point_id] - pods[opp_id].position;
        checkpoint_dir.Normalize();

        if (pods[interceptor].shouldShield(checkpoint_dir, pods[opp_id]))
        {
            out_pod2.sthrust = "SHIELD";
        }
        if (pods[interceptor].shouldShield(pods[5 - opp_id]))
        {
            out_pod2.sthrust = "SHIELD";
        }

    };


private:
    //returns closest point in checkpoint depending on players position 
    //and orientation - finally just checkpoint pos.
    Vector2D getCheckpointPosFromPlayer(const Vector2D& pos,
        const float& angle, const Vector2D& pos_checkpoint)
    {
        return pos_checkpoint;
        /* Vector2D offset = pos_checkpoint - pos;
         float angle_diff = abs(offset.getAngle() * 180.0f / PI - angle);
         if (angle_diff > 90.0f)
         {
             return pos_checkpoint;
         }

         //check if dir intersect circle
         if (offset.Magnitude() * sin(angle_diff * PI / 180.0f) < CHECKPOINT_RADIUS)
         {
             return pos_checkpoint;
         }
         //player direction
         Vector2D dir(cos(angle * PI / 180.0f), sin(angle * PI / 180.0f));
         float dot = dir.DotProduct(offset);
         float diff_outside = (offset.Magnitude() * sin(angle_diff * PI / 180.0f) - (float)CHECKPOINT_RADIUS);
         float angle_outside = atan(diff_outside / dot);
         float angle_diff1 = abs(angle_outside - angle * PI / 180.0f);
         float dist = sqrt(offset.Magnitude() * offset.Magnitude() - ((float)CHECKPOINT_RADIUS - 2) * ((float)CHECKPOINT_RADIUS - 2));
         return Vector2D(dist * cos(angle_diff1), dist * sin(angle_diff1));*/
    }



    //returns distance evaluation
    //for interceptor - recursive - each lvl of recursion = another move of pods
    float PickThrustAngleHelper(const PodData& chaser,
        const PodData& interceptor,
        const PodData& opp1,
        const PodData& opp2,
        const Vector2D& target,
        const vector<float>& angles,
        const vector<float>& thrusts,
        int level
    )
    {

        if (level == 0)
        {

            float ret = getRelativeFitness(chaser, opp1, opp2, false);

            return ret;
        }

        float max_fitness = VERY_SMALL_NUMBER;
        
        float max_fit_shielded = VERY_SMALL_NUMBER;        


        for (int cangle = 0; cangle < angles.size(); ++cangle)
        {
            for (int cthrust = 0; cthrust < thrusts.size(); ++cthrust)
            {
                PodData p1(chaser);
                PodData p2(interceptor);
                PodData o1(opp1);
                PodData o2(opp2);

                float mfit;
                moveAltogether(p1, angles[cangle], thrusts[cthrust],
                    true, p2, o1, o2, mfit);
                int th = thrusts[cthrust];

                Vector2D tar(target);

                tar = checkpoints[p1.check_point_id];


                float fitness = PickThrustAngleHelper(p1, p2, o1, o2, tar,
                    angles, thrusts, level - 1);

                max_fitness = max(max_fitness, fitness);

            }
        }

        return max_fitness;

    }


    //find best target and thrust by simulating real movements 
    void PickThrustAngle(OutputValues& out_pod, const int& lvl)
    {
        vector<float> angles = { -18.0f ,-9.0f, 0.0f, 9.0f, 18.0f };
        vector<float> angles1 = { -18.0f , 0.0f, 18.0f };

        vector<float> thrusts, thrusts1;

        PodData best_pod;
        PodData best_pod_shielded;
        float max_fit_shielded = VERY_SMALL_NUMBER;
        float max_fit = VERY_SMALL_NUMBER;
        float best_thrust = MAX_THRUST;

        int level;

        thrusts1 = { 5.0f, 100.0f };
        thrusts = { 5.0f, 55.0f, 100.0f };
        level = lvl;

        Vector2D offset = checkpoints[pods[0].check_point_id] - pods[0].position;

        for (int cangle = 0; cangle < angles.size(); ++cangle)
        {
            for (int cthrust = 0; cthrust < thrusts.size(); ++cthrust)
            {
                PodData p1(pods[0]);
                PodData p2(pods[1]);
                PodData o1(pods[2]);
                PodData o2(pods[3]);
                float  mfit;
                

                moveAltogether(p1, angles[cangle], thrusts[cthrust],
                    true, p2, o1, o2, mfit);

                max_fit_shielded = max(max_fit_shielded, mfit);

                Vector2D target;
                target = checkpoints[p1.check_point_id];


                float fitness = PickThrustAngleHelper(p1, p2, o1, o2,
                    target, angles, thrusts1, level);
                if (max_fit < fitness)
                {

                    max_fit = fitness;
                    best_pod = p1;
                    best_thrust = thrusts[cthrust];
                }
            }
        }

        if (max_fit_shielded <= max_fit)
        {
            Vector2D dir = best_pod.getDirection();
            Vector2D dd = pods[0].position + dir * 5000.0f;

            out_pod.out_x = (int)dd.x;
            out_pod.out_y = (int)dd.y;
            out_pod.sthrust = std::to_string((int)best_thrust);
        }
        else {
            Vector2D dir = pods[0].getDirection();
            Vector2D dd = pods[0].position + dir * 5000.0f;

            out_pod.out_x = (int)dd.x;
            out_pod.out_y = (int)dd.y;
            out_pod.sthrust = "SHIELD";
        }
    }

    //find best target and thrust by simulating real movements 
    //target is the winning opponent
    void PickThrustAngleInterceptor(OutputValues& out_pod, const Vector2D& target,
        const int& level)
    {

        vector<float> angles = { -18.0f ,-9.0f, 0.0f, 9.0f, 18.0f };
        vector<float> thrusts = { 5.0f, 55.0f, 100.0f };
        PodData best_pod, best_pod_shielded;
        float max_fitness = VERY_SMALL_NUMBER;
        float max_fitness_shielded = VERY_SMALL_NUMBER;
        float best_thrust = MAX_THRUST;


        for (int cangle = 0; cangle < angles.size(); ++cangle)
        {
            for (int cthrust = 0; cthrust < thrusts.size(); ++cthrust)
            {
                PodData p1(pods[1]);
                PodData p2(pods[0]);
                PodData o1(pods[2]);
                PodData o2(pods[3]);
                float m_fitness_shielded;

                moveAltogether(p1, angles[cangle], thrusts[cthrust],
                    false, p2, o1, o2, m_fitness_shielded);

                max_fitness_shielded = max(m_fitness_shielded, max_fitness_shielded);


                float fitness = PickThrustAngleInterceptorHelper(p1, p2, o1, o2,
                    target, angles, thrusts, level);

                if (max_fitness < fitness)
                {

                    max_fitness = fitness;
                    best_pod = p1;
                    best_thrust = thrusts[cthrust];
                }
            }
        }

        if (max_fitness_shielded <= max_fitness)
        {
            Vector2D dir = best_pod.getDirection();
            Vector2D dd = pods[1].position + dir * 5000.0f;

            out_pod.out_x = (int)dd.x;
            out_pod.out_y = (int)dd.y;
            out_pod.sthrust = std::to_string((int)best_thrust);
        }
        else {
            Vector2D dir = pods[1].getDirection();
            Vector2D dd = pods[1].position + dir * 5000.0f;

            out_pod.out_x = (int)dd.x;
            out_pod.out_y = (int)dd.y;
            out_pod.sthrust = "SHIELD";
        }

    }

    //returns distance evaluation
    //for interceptor
    float PickThrustAngleInterceptorHelper(const PodData& pod,
        const PodData& chaser,
        const PodData& op2,
        const PodData& op3,
        const Vector2D& target,
        const vector<float>& angles,
        const vector<float>& thrusts,
        int level)
    {

        if (level == 0)
        {
            float fitness = getRelativeFitnessChaser(chaser, pod, target, op2, op3);
            return fitness;
        }

        float max_fit = VERY_SMALL_NUMBER;
        float max_fit_shielded = VERY_SMALL_NUMBER;
        int picked_thrust;

        for (int cangle = 0; cangle < angles.size(); ++cangle)
        {
            for (int cthrust = 0; cthrust < thrusts.size(); ++cthrust)
            {
                PodData p1(pod);
                PodData p2(chaser);
                PodData o1(op2);
                PodData o2(op3);
                float fit = VERY_SMALL_NUMBER;
                moveAltogether(p1, angles[cangle], thrusts[cthrust],
                    false, p2, o1, o2, fit);
                max_fit_shielded = max(fit, max_fit_shielded);



                float fitness = getRelativeFitnessChaser(p2, p1, target,
                    o1, o2);

                max_fit = max(max_fit, fitness);
                max_fit = max(max_fit, max_fit_shielded);


            }
        }

        return max_fit;

    }

    //moves the pod by angle and thrust
    //max_fit_shielded - suposse to be fitness of shielded pot
    //for the moment its commented
    bool moveAltogether(PodData& p1, const float angle, const float thrust,
        const bool passes_checkpoint,
        PodData& p2, PodData& o1, PodData& o2, float& max_fit_shielded)
    {

        float tt;
        float t[3];
        PodData op1(p1);
        PodData op2(p2);
        PodData oo1(o1);
        PodData oo2(o2);

        p1.addAngle(angle);
        bool col0 = p1.Collide(p2, t[0]);
        bool col2 = p1.Collide(o1, t[1]);
        bool col3 = p1.Collide(o2, t[2]);
        int collider = -1;

        max_fit_shielded = VERY_SMALL_NUMBER;
        /* if( col0 || col2 || col3 )
         {
             float min_t = min(t[0], min(t[1], t[2]));

             if(t[0] == min_t){
                 collider = 0;
             }
             else if(t[1] == min_t){
                 collider = 2;
             } else
             if(t[2] == min_t){
                 collider = 3;
             }
             cerr<< "collider " << collider << endl;
                     //PodData cpod(pods[collider]);
             //p = pod;
             //cerr<< "jestem colli "  << endl;
             if(collider == 0  )
             {
                 //cerr<< "p1.position 1 " << p1.position.x << " " << p1.position.y << endl;

                 p1.moveWithCollision( min_t, thrust, p2);
                 //cerr<< "p1.position after" << p1.position.x << " " << p1.position.y << endl;
                 o1.move(100.0f);
                 o2.move(100.0f);
             }
             else if( collider == 2 )
             {
                 //cerr<< "p1.position 2 " << p1.position.x << " " << p1.position.y << endl;
                 //cerr<< "collider " << collider << endl;
                 p1.moveWithCollision( min_t, thrust, o1);
                 //cerr<< "p1.position after" << p1.position.x << " " << p1.position.y << endl;
                 p2.move(100.0f);
                 o2.move(100.0f);

             } else if( collider == 3 ){
                 //cerr<< "p1.position 2 " << p1.position.x << " " << p1.position.y << endl;
                 p1.moveWithCollision( min_t, thrust, o2);
                 //cerr<< "p1.position after" << p1.position.x << " " << p1.position.y << endl;
                 o1.move(100.0f);
                 p2.move(100.0f);
             }

             //getRelativeFitness(const PodData& p0, cpod, const PodData& p3);
             if(passes_checkpoint && !p1.passed_checkpoint){
                 p1.passed_checkpoint = p1.Collide(checkpoints[p1.check_point_id], Vector2D::Zero(), tt);
                 if(p1.passed_checkpoint)
                 {
                     p1.score++;
                     op1.score++;
                 }
             }
             if( !passes_checkpoint && !p2.passed_checkpoint )
             {
                 p2.passed_checkpoint = p2.Collide(checkpoints[p2.check_point_id], Vector2D::Zero(), tt);
                 if(p2.passed_checkpoint)
                 {
                     p2.score++;
                     op2.score++;
                 }
             }
             if( !passes_checkpoint && !o1.passed_checkpoint )
             {
                 o1.passed_checkpoint = o1.Collide(checkpoints[o1.check_point_id], Vector2D::Zero(), tt);
                 if(o1.passed_checkpoint)
                 {
                     o1.score++;
                     oo1.score++;
                 }
             }
             if( !passes_checkpoint && !o2.passed_checkpoint)
             {
                 o2.passed_checkpoint = o2.Collide(checkpoints[o1.check_point_id], Vector2D::Zero(), tt);
                 if(o2.passed_checkpoint)
                 {
                     o2.score++;
                     oo2.score++;
                 }
             }
             if(!(collider == 0 && !passes_checkpoint))
             {
                 float uthrust = thrust;
                 if( !passes_checkpoint ) //interceptor case - we swap pods
                 {
                     PodData tmp(op1);
                     op1 = op2;
                     op2 = tmp;
                     uthrust = 100.0f;
                 }

                 op1.m = 10.0f;
                 for ( int i =0; i<2; ++i)
                 {
                     op1.move( uthrust );
                     oo1.move( 100.0f );
                     oo2.move( 100.0f );
                     op1.passed_checkpoint = op1.Collide(checkpoints[op1.check_point_id], Vector2D::Zero(), tt);
                     if(op1.passed_checkpoint)
                     {
                         op1.score++;
                     }
                     oo1.passed_checkpoint = oo1.Collide(checkpoints[oo1.check_point_id], Vector2D::Zero(), tt);
                     if(oo1.passed_checkpoint)
                     {
                         oo1.score++;
                     }
                     oo2.passed_checkpoint = oo2.Collide(checkpoints[oo2.check_point_id], Vector2D::Zero(), tt);
                     if(oo2.passed_checkpoint)
                     {
                         oo2.score++;
                     }
                 }
                 float fitness = getRelativeFitness(op1, uthrust, oo1, oo2, false);
                 max_fit_shielded = max(max_fit_shielded, fitness);
             }



         } else */

        {

            p1.move(thrust);
            p2.move(100.0f);
            o1.move(100.0f);
            o2.move(100.0f);

            if (!passes_checkpoint && !p2.passed_checkpoint)
            {
                p2.passed_checkpoint = p2.Collide(checkpoints[p2.check_point_id], Vector2D::Zero(), tt);
                if (p2.passed_checkpoint)
                {
                    p2.check_point_id = (p2.check_point_id + 1) % nr_check_points;
                    p2.score++;
                }
            }
            if (!o1.passed_checkpoint)
            {
                o1.passed_checkpoint = o1.Collide(checkpoints[o1.check_point_id], Vector2D::Zero(), tt);
                if (o1.passed_checkpoint)
                {
                    o1.check_point_id = (o1.check_point_id + 1) % nr_check_points;
                    o1.score++;
                }
            }
            if (!o2.passed_checkpoint)
            {
                o2.passed_checkpoint = o2.Collide(checkpoints[o2.check_point_id], Vector2D::Zero(), tt);
                if (o2.passed_checkpoint)
                {
                    o2.check_point_id = (o2.check_point_id + 1) % nr_check_points;
                    o2.score++;
                }
            }
            if (passes_checkpoint && !p1.passed_checkpoint)
            {
                p1.passed_checkpoint = p1.Collide(checkpoints[p1.check_point_id], Vector2D::Zero(), tt);
                if (p1.passed_checkpoint)
                {
                    p1.check_point_id = (p1.check_point_id + 1) % nr_check_points;
                    p1.score++;
                }
            }



        }
        return col0 || col2 || col3;
    }



    //used for evaluated pods position for interceptor
    //its fitness for chaser and opponnents is modulated by distance of the interceptor
    //to the target
    float getRelativeFitnessChaser(const PodData& chaser, const PodData& p1, const Vector2D& target,
        const PodData& p2, const PodData& p3)
    {
        int opp_id = getWinningOpponentsID();
        float fitness = getRelativeFitness(chaser, p2, p3, false);
        fitness *= 100.0f;
        fitness = -p1.getDistanceEval(target, true, pods[opp_id]);

        return fitness;
    }

    //used for evaluated pods position - the biger the better
    //diffrence betwen score of our chaser and opponents winning pod
    float getRelativeFitness(const PodData& p0,
        const PodData& p2, const PodData& p3, const bool isInterceptor)
    {

        int opp_id = getWinningOpponentsID();
        float scoreChaser = p0.getScore();
        scoreChaser -= p0.getDistanceEval(checkpoints[p0.check_point_id], false, pods[opp_id]);


        //we assume that opponnents move with thrust 100
        float score2 = p2.getScore();
        score2 -= p2.getDistanceEval(checkpoints[p2.check_point_id], false, pods[opp_id]);

        float score3 = p3.getScore();
        score3 -= p3.getDistanceEval(checkpoints[p3.check_point_id], false, pods[opp_id]);

        float opponent_score = max(score2, score3);
        return scoreChaser - opponent_score;
    }



    //set the target for interceptor
    Vector2D setTargetForInterceptor(const PodData& pod, int opp_id)
    {
        //find checkpoint where interceptor is closer then 
        //opponent
        const int opp_check = pods[opp_id].check_point_id;
        Vector2D offset1 = pods[opp_id].position - pods[1].position;
        float dot = pods[opp_id].speed.DotProduct(pods[1].speed);
        bool well_oriented = pods[1].speed.DotProduct(offset1) > 0.0f;
        if (well_oriented && dot < -0.9f && offset1.Magnitude() < 4000.0f)
        {
            return pods[opp_id].position;
        }
        int ch_id = -1;
        for (int i = opp_check; i < nr_check_points + opp_check && ch_id == -1; ++i)
        {
            Vector2D offset_pl = checkpoints[i % nr_check_points] - pods[1].position;
            Vector2D offset_opp = checkpoints[i % nr_check_points] - pods[opp_id].position;
            if (offset_pl.Magnitude() < offset_opp.Magnitude())
            {
                ch_id = i % nr_check_points;
            }
        }

        if (ch_id == -1)
        {
            return pods[opp_id].position;
        }

        Vector2D offset = checkpoints[ch_id % nr_check_points] - pods[1].position;
        if (INTERCEPTOR_SEARCH_TRESHOLD > offset.Magnitude())
        {

            int nr_moves = offset1.Magnitude() / 750.0f;
            cerr << " nr_moves " << nr_moves << endl;
            PodData opp(pods[opp_id]);
            for (int i = 0; i < nr_moves + 1; ++i)
            {
                opp.move(100);
            }
            return opp.position;
        }
        else {
            return checkpoints[ch_id % nr_check_points];
        }

    }


    //id of the winning opponents - important for pod  intercepting
    // we check nr of pods passed and then distance to pod
    int getWinningOpponentsID()
    {
        int score2 = pods[2].check_point_id + pods[2].lap * nr_check_points;
        int score3 = pods[3].check_point_id + pods[3].lap * nr_check_points;

        if (score2 > score3)
        {
            return 2;
        }
        else if (score2 < score3)
        {
            return 3;
        }
        else {
            const Vector2D& checkpoint_pos = checkpoints[pods[2].check_point_id];
            Vector2D offset2 = checkpoint_pos - pods[2].position;
            Vector2D offset3 = checkpoint_pos - pods[3].position;
            if (offset2.Magnitude() > offset3.Magnitude())
            {
                return 3;
            }
            else
            {
                return 2;
            }
        }
    }

    //vector from one checkpoint to another
    Vector2D getRoute(const int id)
    {
        return (checkpoints[id] - checkpoints[(id - 1) % nr_check_points]);
    }

};


/**
 * Auto-generated code below aims at helping you parse
 * the standard input according to the problem statement.
 **/

int main()
{
    vector<Vector2D> checkpoints;
    int laps;
    cin >> laps; cin.ignore();
    int checkpoint_count;
    cin >> checkpoint_count; cin.ignore();
    for (int i = 0; i < checkpoint_count; i++) {
        int checkpoint_x;
        int checkpoint_y;
        cin >> checkpoint_x >> checkpoint_y; cin.ignore();
        cerr << "checkpoint_x  checkpoint_y " << checkpoint_x << " " << checkpoint_y << endl;
        checkpoints.push_back(Vector2D(checkpoint_x, checkpoint_y));
    }
    Game game(checkpoints, laps);
    bool first = true;

    // game loop
    while (1) {
        for (int i = 0; i < 2; i++) {
            int x; // x position of your pod
            int y; // y position of your pod
            int vx; // x speed of your pod
            int vy; // y speed of your pod
            int angle; // angle of your pod
            int next_check_point_id; // next check point id of your pod
            cin >> x >> y >> vx >> vy >> angle >> next_check_point_id; cin.ignore();
            cerr << " next_check_point_id " << std::to_string(next_check_point_id) << endl;
            //PodData podData( Vector2D(x, y), Vector2D(vx, vy), angle, next_check_point_id );
            game.UpdatePod(Vector2D(x, y), Vector2D(vx, vy), angle, next_check_point_id, i);
        }
        for (int i = 0; i < 2; i++) {
            int x_2; // x position of the opponent's pod
            int y_2; // y position of the opponent's pod
            int vx_2; // x speed of the opponent's pod
            int vy_2; // y speed of the opponent's pod
            int angle_2; // angle of the opponent's pod
            int next_check_point_id_2; // next check point id of the opponent's pod
            cin >> x_2 >> y_2 >> vx_2 >> vy_2 >> angle_2 >> next_check_point_id_2; cin.ignore();
            //cerr << " next_check_point_id2 " << std::to_string(next_check_point_id_2) << endl;
            PodData podData(Vector2D(x_2, y_2), Vector2D(vx_2, vy_2), angle_2, next_check_point_id_2);
            game.UpdatePod(Vector2D(x_2, y_2), Vector2D(vx_2, vy_2), angle_2, next_check_point_id_2, i + 2);
        }

        OutputValues out_pod1;
        OutputValues out_pod2;
        //cout << out_pod1.out_x << " " << out_pod1.out_y << " " << "100" << endl;
        //cout << out_pod1.out_x << " " << out_pod1.out_y << " " << "100" << endl;
        game.ComputeValues(out_pod1, out_pod2, first);
        // Write an action using cout. DON'T FORGET THE "<< endl"
        // To debug: cerr << "Debug messages..." << endl;


        // You have to output the target position
        // followed by the power (0 <= thrust <= 100)
        // i.e.: "x y thrust"
        //cerr << " out_pod1.out_x " << std::to_string(out_pod1.out_x) << endl;
        //cerr << " out_pod1.out_y " << out_pod1.out_y << endl;
        //cerr << " out_pod1.sthrust " << out_pod1.sthrust << endl;
        //cout << "8000 4500 100" << endl;
        //cout << "8000 4500 100" << endl;
        cout << out_pod1.out_x << " " << out_pod1.out_y << " " << out_pod1.sthrust << endl;
        cout << out_pod2.out_x << " " << out_pod2.out_y << " " << out_pod2.sthrust << endl;

    }
}

