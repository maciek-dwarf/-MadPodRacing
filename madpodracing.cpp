
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <limits>


using namespace std;

/**
 * Auto-generated code below aims at helping you parse
 * the standard input according to the problem statement.
 **/
namespace {
    constexpr int SLOW_DAWN_TRESHOLD = 1800;

    constexpr int BOOST_DISTANCE_TRESHOLD = 4000;
    constexpr int LONG_ROUTE_TRESHOLD = 4000;

    constexpr float ANGLE_TRESHOLD = 80.0f / 90.0f;
    constexpr int MAX_THRUST = 100;
    constexpr int MIN_THRUST = 60;
    constexpr int MIN_PTHRUST = 30;
    constexpr int PREV_SLOW_DAWN_TRESHOLD = 1500;
    constexpr int COLLISION_TRESHOLD = 2500;
    constexpr int PLAYER_COLLISION_TRESHOLD = 2100;
    constexpr float CHECKPOINT_RADIUS = 600;
    constexpr float POD_RADIUS = 400;

    constexpr int COLLISIONS_ANGLE_TRESHOLD = 15;
    constexpr float PI = 3.14159265359f;
    constexpr float SHOULD_SLOWDAWN_ANGLE_TRESHOLD = 60.0f;
    constexpr int SIMULATION_LEVEL = 3;
    constexpr int OPPONENT_SIMULATION_LEVEL = 2;
    constexpr float SPEED_TO_SHIELD = 120.0f;
    constexpr float DOT_TO_SHIELD = 0.8f;
    constexpr float BOOST_ANGLE_TRESHOLD = 10.0f * PI / 180.0f;
    constexpr float FORCE_THRUST_TRESHOLD = 2000;

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
        passed_checkpoint(false)
    {
    }

    PodData() = default;
    PodData(const PodData&) = default;

    void UpdatePod(const Vector2D& pos, const Vector2D& s, const int a, const int ch_point_id)
    {
        position = pos;
        speed = s;
        if (check_point_id != ch_point_id && ch_point_id == 0)
        {
            lap++;
        }
        check_point_id = ch_point_id;
        angle = a;

    }

    Vector2D getDirection() const
    {
        return Vector2D(cos(angle * PI / 180.0f), sin(angle * PI / 180.0f));
    }

    void addAngle(float angleDiff) {
        angle += angleDiff;
        //cerr<< " angle = a " << angle << endl;
        if (angle < 0.0f) {
            angle = 360.0f + angle;
        }
        else if (angle > 360.0f) {
            angle = angle - 360.0f;
        }
    }

    //used for simulation - moves the pod
    //following expert's rules
    float move(float thrust, float angleDiff, bool debug = false)
    {
        Vector2D prevpos(position);
        addAngle(angleDiff);

        float rangle = angle * (PI / 180.0f);

        speed += Vector2D((float)thrust * cos(rangle), (float)thrust * sin(rangle));

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
        bool collide = Collide(pod.position, 1.5f * POD_RADIUS);
        Vector2D opp_pod_dir = pod.getDirection();

        if (collide)
            cerr << " collide 2 " << endl;

        if (collide && mspeed > SPEED_TO_SHIELD)
        {
            return true;
        }
        return false;
    }



    bool shouldShield(const Vector2D& checkpoint_dir, const PodData& pod) const
    {
        float mspeed = speed.Magnitude();
        bool collide = Collide(pod.position, 1.5f * POD_RADIUS);
        Vector2D opp_pod_dir = pod.getDirection();
        /* cerr << " checkpoint_dir " << checkpoint_dir.x << " " << checkpoint_dir.y << endl;
         cerr << " abs(opp_pod_dir1.DotProduct(checkpoint_dir) " << opp_pod_dir.DotProduct(checkpoint_dir) << endl;
         cerr << " pods[1].angle *PI/180.0f " << pod.angle * PI / 180.0f << endl;
         cerr << " angle *PI/180.0f " << angle * PI / 180.0f << endl;
         cerr << " mspeed " << mspeed << endl;*/

        if (collide)
            cerr << " collide 1 " << endl;

        if (collide && mspeed > SPEED_TO_SHIELD && (opp_pod_dir.DotProduct(checkpoint_dir))
            < DOT_TO_SHIELD)
        {
            return true;
        }
        return false;
    }

    bool Collide(const Vector2D& pos, const float collision_radius) const
    {
        Vector2D offset = position - pos;
        //cerr << " offset.Magnitude() " << offset.Magnitude() << endl;
        return  offset.Magnitude() < (collision_radius + POD_RADIUS);
    }

public:
    Vector2D position;
    Vector2D speed;
    int check_point_id;
    float angle;
    int lap;
    bool passed_checkpoint;

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
    int longest_route_id;

public:
    Game(const vector<Vector2D>& chpoints, int l) :
        checkpoints(chpoints), laps(l), has_boost(true)
    {
        pods.resize(4);
        nr_check_points = checkpoints.size();

        longest_route_id = getLongestRouteID();
        cerr << "longest_route_id " << longest_route_id << endl;
        ComputeShouldSlowdawn();
    }
    //returns closest point in checkpoint depending on players position 
    //and orientation
    Vector2D getCheckpointPosFromPlayer(const Vector2D& pos,
        const float& angle, const Vector2D& pos_checkpoint)
    {
        return pos_checkpoint;
        Vector2D offset = pos_checkpoint - pos;
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
        return Vector2D(dist * cos(angle_diff1), dist * sin(angle_diff1));
    }


    //returns distance evaluation
    float PickThrustAngleHelper(const PodData& pod, int& pthrust,
        const Vector2D& target,
        const vector<float>& angles, 
        const vector<float>& thrusts, 
        int level, bool chaser, 
        bool debug = false)
    {

        Vector2D offset = target - pod.position;

        float dist = offset.Magnitude();
        if (level == 0)
        {
            offset.Normalize();
            Vector2D dir = pod.getDirection();
            //(cos(pod.angle * PI / 180.0f), sin(pod.angle * PI / 180.0f));

            float angle = abs(acos(offset.DotProduct(dir)));
            //return dist + pod.speed.Magnitude() * angle * angle * 10.0f;
            if (pod.passed_checkpoint)
            {
                return dist + pod.speed.Magnitude() * angle * angle * 10.0f;
            }
            else {
                return dist;
            }
        }

        float min_dist = numeric_limits<float>::max();
        int picked_thrust;
        bool passed_checkpoint = pod.passed_checkpoint;
        for (int cangle = 0; cangle < angles.size(); ++cangle)
        {
            for (int cthrust = 0; cthrust < thrusts.size(); ++cthrust)
            {
                PodData p(pod);
                int th = thrusts[cthrust];
                float dist = p.move(thrusts[cthrust], angles[cangle], false);
                Vector2D t(target);
                
                if (p.passed_checkpoint)
                {
                    t = checkpoints[(pod.check_point_id + 1) % nr_check_points];
                }
                else {
                    t = checkpoints[pod.check_point_id];
                }
                float dist_eval = PickThrustAngleHelper(p, th, t,
                    angles, thrusts, level - 1, chaser, cangle == 0);
                if (!passed_checkpoint && p.passed_checkpoint)
                {

                    passed_checkpoint = true;
                    picked_thrust = thrusts[cthrust];
                    min_dist = dist_eval;


                } 
                else if (min_dist > dist_eval && passed_checkpoint == p.passed_checkpoint)
                {
                    min_dist = dist_eval;
                    picked_thrust = thrusts[cthrust];
                }
                
            }
        }

        if (passed_checkpoint)
        {
            min_dist -= 1000000.0f;
        }

        if (level == SIMULATION_LEVEL)
        {
            //pthrust = picked_thrust;
            //cerr<<"pthrust "<< pthrust<<  " picked_thrust " <<picked_thrust<<endl;
        }
        return min_dist;

    }


    //find best target and thrust by simulating real movements 
    void PickThrustAngle(OutputValues& out_pod, const PodData& pod, const int& lvl)
    {
        vector<float> angles = { -18.0f ,-9.0f, 0.0f, 9.0f, 18.0f };
        vector<float> angles1 = { -18.0f , 0.0f, 18.0f };

        vector<float> thrusts;
        //vector<float> thrusts1 = { 100.0f };
        PodData best_pod;
        float min_dist = numeric_limits<float>::max();
        float best_thrust = MAX_THRUST;
        bool passed_checkpoint = false;        
        int level;
        
        thrusts = {5.0f, 55.0f, 100.0f };
        level = lvl;
        cerr << " not force  " << endl;
        
        //cerr<< " pod.angle  " << pod.angle << endl;
        for (int cangle = 0; cangle < angles.size(); ++cangle)
        {
            for (int cthrust = 0; cthrust < thrusts.size(); ++cthrust)
            {
                PodData p(pod);
                float dist = p.move(thrusts[cthrust], angles[cangle], cangle == 0);
                p.passed_checkpoint = p.Collide(checkpoints[p.check_point_id], CHECKPOINT_RADIUS);
                Vector2D target;
                if (p.passed_checkpoint)
                {
                    //cerr<< "checked 1 " << endl; 
                    target = checkpoints[(pod.check_point_id + 1) % nr_check_points];
                    target = getCheckpointPosFromPlayer(pod.position, pod.angle, target);
                }
                else {
                    target = checkpoints[pod.check_point_id];
                    target = getCheckpointPosFromPlayer(pod.position, pod.angle, target);
                }
                int th = thrusts[cthrust];
                float dist_eval = PickThrustAngleHelper(p, th, target, angles, thrusts, level, false);

                if (!passed_checkpoint && p.passed_checkpoint)
                {

                    passed_checkpoint = true;
                    min_dist = dist_eval;
                    best_pod = p;
                    best_thrust = th;//thrusts[cthrust];
                }
                else if (min_dist > dist_eval && passed_checkpoint == p.passed_checkpoint)
                {
                    min_dist = dist_eval;
                    best_pod = p;
                    best_thrust = th;// thrusts[cthrust];                    
                }
            }
        }

        Vector2D dir = best_pod.getDirection();
        Vector2D dd = pod.position + dir * 5000;
        
        out_pod.out_x = (int)dd.x;
        out_pod.out_y = (int)dd.y;
        
        cerr << "best_thrust " << best_thrust << endl;
        out_pod.sthrust = std::to_string((int)best_thrust);
    }

    //find best target and thrust by simulating real movements 
    //target is the winning opponent
    void PickThrustAngleForChaser(OutputValues& out_pod, const PodData& pod, const Vector2D& target,
        const int& level)
    {

        vector<float> angles = { -18.0f ,-9.0f, 0.0f, 9.0f, 18.0f };
        vector<float> thrusts = { 5.0f, 50.0f, 100.0f };
        PodData best_pod;
        float min_dist = numeric_limits<float>::max();
        float best_thrust = MAX_THRUST;

        for (int cangle = 0; cangle < angles.size(); ++cangle)
        {
            for (int cthrust = 0; cthrust < thrusts.size(); ++cthrust)
            {
                PodData p(pod);
                float dist = p.move(thrusts[cthrust], angles[cangle], cangle == 0);
                int th = thrusts[cthrust];
                float dist_eval = PickThrustAngleChaserHelper(p, th, target,
                    angles, thrusts, level);

                if (min_dist > dist_eval)
                {
                    min_dist = dist_eval;
                    best_pod = p;
                    best_thrust = thrusts[cthrust];
                }

            }
        }
        Vector2D dir = best_pod.getDirection();
        Vector2D dd = pod.position + dir * 5000;
        
        out_pod.out_x = (int)dd.x;
        out_pod.out_y = (int)dd.y;
        cerr << "out_pod.out_x " << out_pod.out_x << endl;
        out_pod.sthrust = std::to_string((int)best_thrust);
    }

    //returns distance evaluation
    float PickThrustAngleChaserHelper(const PodData& pod, int& pthrust,
        const Vector2D& target,
        const vector<float>& angles, 
        const vector<float>& thrusts, 
        int level)
    {

        Vector2D offset = target - pod.position;

        float dist = offset.Magnitude();
        if (level == 0)
        {
            offset.Normalize();
            Vector2D dir = pod.getDirection();
            //(cos(pod.angle * PI / 180.0f), sin(pod.angle * PI / 180.0f));

            float angle = abs(acos(offset.DotProduct(dir)));
            //return dist + pod.speed.Magnitude() * angle * angle * 10.0f;
            
            return dist;
            
        }

        float min_dist = numeric_limits<float>::max();
        int picked_thrust;
        
        for (int cangle = 0; cangle < angles.size(); ++cangle)
        {
            for (int cthrust = 0; cthrust < thrusts.size(); ++cthrust)
            {
                PodData p(pod);
                int th = thrusts[cthrust];
                float dist = p.move(thrusts[cthrust], angles[cangle], false);
                Vector2D t(target);
                
                float dist_eval = PickThrustAngleChaserHelper(p, th, t, angles, thrusts, level - 1);
                if (min_dist > dist_eval)
                {
                    min_dist = dist_eval;
                }
                
            }
        }
        
        if (level == SIMULATION_LEVEL)
        {
            //pthrust = picked_thrust;
            //cerr<<"pthrust "<< pthrust<<  " picked_thrust " <<picked_thrust<<endl;
        }
        return min_dist;

    }


    //checks if we should slow dawn when aiming
    void ComputeShouldSlowdawn()
    {
        should_slowdawn.resize(nr_check_points);
        for (int i = 0; i < nr_check_points; ++i)
        {
            Vector2D route1 = getRoute(i);
            Vector2D route2 = getRoute(i + 1);
            float angle1 = route1.getAngle();
            float angle2 = route2.getAngle();
            if (abs(angle1 - angle2) < ((SHOULD_SLOWDAWN_ANGLE_TRESHOLD * PI) / 180.0f))
            {
                should_slowdawn[i] = false;
            }
            else {
                should_slowdawn[i] = true;
            }

        }
    }

    Game() = default;


    void UpdatePod(const Vector2D& pos, const Vector2D& s, const int check_point_id, const int a, const int id)
    {
        pods[id].UpdatePod(pos, s, check_point_id, a);
        //        cerr << "angle pod " << a << endl;

    }

    Vector2D setTargetForInterceptor(int opp_id)
    {
        Vector2D dir = pods[1].getDirection();
        Vector2D offset_checkpoint1 = checkpoints[pods[opp_id].check_point_id] - pods[opp_id].position;
        Vector2D offset_checkpoint2 = checkpoints[pods[opp_id].check_point_id] - pods[1].position;
        bool first = offset_checkpoint2.Magnitude() > offset_checkpoint1.Magnitude();
        offset_checkpoint1.Normalize();
        first = (offset_checkpoint1.DotProduct(dir)) > 0.9f && first;
        cerr << " abs(offset_checkpoint1.DotProduct(dir)) " << (offset_checkpoint1.DotProduct(dir)) << endl;
        if (!first)
        {
            /*PodData opp(pods[opp_id]);
            for (int i = 0; i < OPPONENT_SIMULATION_LEVEL + 5; ++i)
            {
                opp.move(100, 0);
            }
            return opp.position;*/
            Vector2D dir2 = pods[opp_id].getDirection();
            Vector2D pos = pods[1].position - pods[opp_id].position;
            float dot = pos.DotProduct(dir2);
            Vector2D target = dir2 * 0.5f * pos.Magnitude() * pos.Magnitude() / dot;
            cerr << "DIR 1" << endl;
            return target + pods[opp_id].position;
        }
        else
        {
            cerr << "DIR 2" << endl;
            cerr << "pods[opp_id].position " << pods[opp_id].position.x << " " << pods[opp_id].position.y
                << endl;
            return pods[opp_id].position;
        }
    }


    //id of the winning opponents - important for pod  intercepting
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

    //find id of the longest route - used to compute boost
    int getLongestRouteID()
    {
        int id = 0;
        float longest_route = std::numeric_limits<float>::min();
        for (int i = 0; i < nr_check_points; ++i)
        {
            Vector2D route = getRoute(i);
            float longeur = getRoute(i).Magnitude();
            if (longest_route < longeur)
            {
                id = i;
                longest_route = longeur;
            }
        }
        return id;
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

            PickThrustAngle(out_pod1, pods[chaser], SIMULATION_LEVEL);
            cerr << " thrust 11111 " << out_pod1.sthrust << endl;
            //second pod will try to intercept winning opponent

            Vector2D target = setTargetForInterceptor(opp_id);
            PickThrustAngleForChaser(out_pod2, pods[interceptor], target, OPPONENT_SIMULATION_LEVEL);
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
        /*Vector2D opp_pod_dir1 = pods[interceptor].getDirection();

        float mspeed = pods[chaser].speed.Magnitude();
        bool collide = pods[chaser].Collide(pods[interceptor].position, 1.2f * POD_RADIUS);
        cerr << " abs(opp_pod_dir1.DotProduct(checkpoint_dir) " << abs(opp_pod_dir1.DotProduct(checkpoint_dir)) << endl;
        cerr << " pods[1].angle *PI/180.0f " << pods[interceptor].angle * PI / 180.0f << endl;
        if (collide)
            cerr << " collide 1 " << endl;
        if (collide && mspeed > SPEED_TO_SHIELD && (opp_pod_dir1.DotProduct(checkpoint_dir))
            < DOT_TO_SHIELD) {
            out_pod1.sthrust = "SHIELD";
        }*/

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

        //computing boost
        if (has_boost)
        {
            /*    float angle = offset.getAngle();
                float checkpoint_angle = angle - ((float)pods[chaser].angle * PI / 180.0f);
                cerr << " thruuustttt longest_route_id " << longest_route_id << " pods[0].check_point_id "
                    << pods[chaser].check_point_id << endl;
                cerr << " std::abs(checkpoint_angle) " << std::abs(checkpoint_angle) << " angle " << angle << endl;
                //use boost on first pod if we are on the longest route betwen 2 checkpoints
                bool use_boost = pods[chaser].check_point_id == longest_route_id &&
                    std::abs(checkpoint_angle) < BOOST_ANGLE_TRESHOLD && checkpoint_dist > BOOST_DISTANCE_TRESHOLD;
                if (use_boost)
                {
                    cerr << "boost " << endl;
                    out_pod1.sthrust = "BOOST";
                    has_boost = false;
                }
            */
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
            //cerr << " next_check_point_id " << std::to_string(next_check_point_id) << endl;
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

