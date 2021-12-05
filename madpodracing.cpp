
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
    constexpr int OPPONENT_SIMULATION_LEVEL = 3;
    constexpr float SPEED_TO_SHIELD = 120.0f;
    constexpr float DOT_TO_SHIELD = 0.8f;
    constexpr float BOOST_ANGLE_TRESHOLD = 10.0f * PI / 180.0f;
    constexpr float FORCE_THRUST_TRESHOLD = 2000.0f;
    constexpr float BIG_NUMBER = 30000.0f;
    constexpr float VERY_SMALL_NUMBER = -1000000000.0f;

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
            
            
            cerr<< " jestem " << score << endl;
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

    bool UCollide(const PodData& p, float& t) const
    {
        t = numeric_limits<float>::max();
        Vector2D soffset = p.speed - speed;
        Vector2D poffset = p.position - position;
        float a = soffset.DotProduct(soffset);
        float b = soffset.DotProduct(poffset) * 2.0f;
        float c = poffset.DotProduct(poffset) - 4.0f * (float)(POD_RADIUS * POD_RADIUS*1.2f);
        float rc = b*b -4 * a *c;
        if(rc<0 || a == 0.0f)
        {
            return false;
        }    
        rc = sqrt(rc);
        float x1 = (-b-rc)/(2.0f*a);
        float x2 = (-b+rc)/(2.0f*a);
        if(x1>=0.0f )
        {
            t = x1;
            return true;
        }
        if(x2>=0.0f )
        {
            t = x2;
            return true;
        }

        //cerr << " offset.Magnitude() " << offset.Magnitude() << endl;
        return false;
    }

    bool Collide(const PodData& p, float& t) const
    {
        t = numeric_limits<float>::max();
        Vector2D soffset = p.speed - speed;
        Vector2D poffset = p.position - position;
        float a = soffset.DotProduct(soffset);
        float b = soffset.DotProduct(poffset) * 2.0f;
        float c = poffset.DotProduct(poffset) - 4.0f * (float)(POD_RADIUS * POD_RADIUS*1.2f);
        float rc = b*b -4 * a *c;
        if(rc<0 || a == 0.0f)
        {
            return false;
        }    
        rc = sqrt(rc);
        float x1 = (-b-rc)/(2.0f*a);
        float x2 = (-b+rc)/(2.0f*a);
        if(x1>=0.0f && x1<=1.0f)
        {
            t = x1;
            return true;
        }
        if(x2>=0.0f && x2<=1.0f)
        {
            t = x2;
            return true;
        }

        //cerr << " offset.Magnitude() " << offset.Magnitude() << endl;
        return false;
    }
    //used to check collision with checkpoint
    bool Collide(const Vector2D& pposition, const Vector2D& pspeed, float& t) const
    {
        Vector2D soffset = speed;
        Vector2D poffset = pposition - position;
        if(poffset.Magnitude() < (float)(CHECKPOINT_RADIUS))
        {
            return true;
        }
        float dot = soffset.DotProduct(poffset) /soffset.Magnitude();

        if(dot < 0.0f)
        {
            return false;
        }
        if( poffset.DotProduct(poffset) - dot * dot > (float)(CHECKPOINT_RADIUS*CHECKPOINT_RADIUS))
        {
            return false;
        }
        return soffset.Magnitude()>dot; 
        /*float a = soffset.DotProduct(soffset);
        float b = soffset.DotProduct(poffset) * 2.0f;
        float c = poffset.DotProduct(poffset) -  (float)(CHECKPOINT_RADIUS) 
            * ( CHECKPOINT_RADIUS );
        float rc = b*b -4 * a *c;
        if(rc<0 || a == 0.0f)
        {
            return false;
        }    
        rc = sqrt(rc);
        float x1 = (-b-rc)/(2.0f*a);
        float x2 = (-b+rc)/(2.0f*a);
        if(x1>=0.0f && x1<=1.0f)
        {
            t = x1;
            return true;
        }
        if(x2>=0.0f && x2<=1.0f)
        {
            t = x2;
            return true;
        }

        //cerr << " offset.Magnitude() " << offset.Magnitude() << endl;*/
        return false;
    }

    //simulation of collision
    //of 2 pods
    void CollisionSimulation(const PodData& pod, Vector2D& speed1, Vector2D& speed2) const
    {
        //direction 
        Vector2D u = (pod.position - position);
        //i assume that pod's mass is shielded   - 10.0f      
        float mm = (m*10.0f) / (m+10.0f);
        float fact = (pod.speed - speed).DotProduct(u);
        float impulse = -2*mm*fact;
        if(impulse <120.0f)
        {
            impulse = 120.0f;
        }
        //impulse = std::clamp(impulse, -120.0f, 120.0f);
        speed1 = speed;
        speed1 += u * (1/m) * impulse;
        speed2 = pod.speed;
        speed2 += u * (-1/10.0f) * impulse;
    }

    void moveWithCollision( const float t, const float thrust, 
         PodData& cpod)
    {
        
        //addAngle(angleDiff);

        
        Vector2D dir = getDirection();
        Vector2D cdir = cpod.getDirection(); 

        
        //we assume that with m =10 (shielded) speed is not updated
        //we assume that colided pod has mas 10 is shielded
        //cpod.speed += cdir * 100.0f;
        if(m < 9.0f){        
            speed += dir * (float)thrust;
        }
    
        position += speed*t;
        cpod.position += cpod.speed*t;

        Vector2D speed1, speed2;
        CollisionSimulation( cpod, speed1, speed2);
        speed = speed1;
        cpod.speed = speed2;

        position += speed*(1.0f-t);
        cpod.position += cpod.speed*(1.0f-t);

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
        if(m < 9.0f){
            speed += dir * (float)thrust;
        }
        //Vector2D((float)thrust * cos(rangle), (float)thrust * sin(rangle));

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
        float t;
        bool collide = Collide(pod, t);
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

    float getScore() const
    {        
        return (float)score*BIG_NUMBER;
    }

    float getDistanceEval(const Vector2D& target, const bool isInterceptor) const
    {
        Vector2D offset = target - position;
        float dist = offset.Magnitude();
        offset.Normalize();
        //Vector2D dir = getDirection();            
        float angle1 = offset.getAngle();
        //abs(acos(offset.DotProduct(dir)));
        float diff_angle = angle * PI/180.0f- angle1;
        
        //return dist + pod.speed.Magnitude() * angle * angle * 10.0f;

        //cerr<< " dist " << dist<< endl;
        if(!isInterceptor && passed_checkpoint)
        {
           dist+= speed.Magnitude() * abs(diff_angle) * 3.0f;
        }
        return dist; //+speed.Magnitude() * abs(diff_angle) * 50.0f;
                                

    }
    /*bool Collide(const Vector2D& pos, const float collision_radius) const
    {
        Vector2D offset = position - pos;
        //cerr << " offset.Magnitude() " << offset.Magnitude() << endl;
        return  offset.Magnitude() < (collision_radius + POD_RADIUS);
    }*/

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

    //float 

    //returns distance evaluation
    float PickThrustAngleHelper(const PodData& chaser, 
                                const PodData& interceptor, 
                                const PodData& opp1,
                                const PodData& opp2,
                                int& pthrust,
                                const Vector2D& target,
                                const vector<float>& angles, 
                                const vector<float>& thrusts, 
                                int level 
        )
    {
        

        //Vector2D offset = target - pod.position;
        //cerr<< " pod.score " << pod.score << " pod.angle "<< pod.angle<< endl;
        //float dist = offset.Magnitude();
        if (level == 0)
        {
            /*offset.Normalize();
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
            }*/
            float ret = getRelativeFitness(chaser, pthrust ,opp1, opp2, false );
            //cerr << "pod.getScore() " << pod.getScore() << " fitness " << ret<< endl;
           // cerr<< " ret = getRelativeFitness(pod, opp1, opp2 ) " << ret<<endl;
            return ret;
        }

        float max_fitness = VERY_SMALL_NUMBER;//numeric_limits<float>::min();
        int picked_thrust;
        //bool passed_checkpoint = chaser.passed_checkpoint;
        float max_fit_shielded = VERY_SMALL_NUMBER;
        vector<float> thrusts1;
        thrusts1 = thrusts;
        /*if( level < 2 )
        {
            thrusts1 =  { 5.0f, 100.0f };
        } else 
        {
            
        }*/
         

        for (int cangle = 0; cangle < angles.size(); ++cangle)
        {
            for (int cthrust = 0; cthrust < thrusts1.size(); ++cthrust)
            {
                PodData p1(chaser); 
                PodData p2(interceptor);
                PodData o1(opp1);
                PodData o2(opp2);          
                float tttt;
                bool col3 = p1.Collide( o2, tttt);
                
                
                /*p1.addAngle(angles[cangle]);
                p1.move(thrusts[cthrust]);
                float tt;
                p1.passed_checkpoint = p1.Collide(checkpoints[p1.check_point_id], Vector2D::Zero(), tt);
                if(p1.passed_checkpoint)                
                {
                    //cerr<< "o2.score , p1.score " << o2.score << " " <<  p1.score <<endl;                    
                    p1.check_point_id = (p1.check_point_id+1)% nr_check_points;
                    //p1.score++;                 
                }*/
              //  moveAltogether(p1, angles[cangle], thrusts[cthrust],
                //        true, p2, o1, o2, max_fit_shielded, chaser.passed_checkpoint);
                //max_fitness = max(max_fitness, max_fit_shielded);
                p1.addAngle(angles[cangle]);
                p1.move(thrusts[cthrust]);
                if( !p1.passed_checkpoint )
                {   
                    float tt;             
                    
                    if(p1.passed_checkpoint)                
                    {
                        
                        p1.check_point_id = (p1.check_point_id+1)% nr_check_points;
                        p1.score++;                 
                        cerr<<  " p1.score helper " <<  p1.score << " chaser.score " <<chaser.score <<endl;
                    }
                }

                //float fitness = getRelativeFitness(p, thrusts[cthrust], o1, o2 );
                //max_fitness = max(max_fitness, fitness);
                int th = thrusts[cthrust];
                //float dist = p.move(thrusts[cthrust]);
                Vector2D tar(target);
                
                /*if (p1.passed_checkpoint)
                {
                    //cerr<<" HEREEE " << (pod.check_point_id + 1) % nr_check_points << endl; 
                    
                    tar = checkpoints[(pod.check_point_id + 1) % nr_check_points];
                    //cerr<< "tar1 " << tar.x << " " <<tar.y<< endl;
                }
                else*/ {
                    tar = checkpoints[p1.check_point_id];
                    //cerr<< "tar2 " << tar.x << " " <<tar.y<< endl;
                }
                float fitness =  PickThrustAngleHelper(p1, p2, o1, o2, th, tar,
                       angles, thrusts1, level - 1);
                
                //cerr<< " fitness " << fitness << " max_fitness " << max_fitness << endl;
                max_fitness = max(max_fitness, fitness);
              
                //cerr<< " fitness " << fitness << " max_fitness " << max_fitness << endl;
                /*if (!passed_checkpoint && p.passed_checkpoint)
                {

                    passed_checkpoint = true;
                    picked_thrust = thrusts[cthrust];
                    min_dist = dist_eval;


                } 
                else if (min_dist > dist_eval && passed_checkpoint == p.passed_checkpoint)
                {
                    min_dist = dist_eval;
                    picked_thrust = thrusts[cthrust];
                }*/
                
                
            }
        }

       

        if (level == SIMULATION_LEVEL)
        {
            //pthrust = picked_thrust;
            //cerr<<"pthrust "<< pthrust<<  " picked_thrust " <<picked_thrust<<endl;
        }
        //cerr<< "return max_fitness; " << max_fitness <<endl;
        return max_fitness;

    }


    //find best target and thrust by simulating real movements 
    void PickThrustAngle(OutputValues& out_pod, const PodData& pod, const int& lvl)
    {
        vector<float> angles = { -18.0f ,-9.0f, 0.0f, 9.0f, 18.0f };
        vector<float> angles1 = { -18.0f , 0.0f, 18.0f };

        vector<float> thrusts, thrusts1;
        //vector<float> thrusts1 = { 100.0f };
        PodData best_pod;
        PodData best_pod_shielded;
        float max_fit_shielded = VERY_SMALL_NUMBER;
        float max_fit = VERY_SMALL_NUMBER ; //numeric_limits<float>::min();
        float best_thrust = MAX_THRUST;
        bool passed_checkpoint = false;        
        int level;
        
        thrusts1 = {5.0f, 100.0f };
        thrusts = {5.0f, 55.0f, 100.0f };
        level = lvl;
        //cerr << " not force  " << endl;
        //cerr<<" pods[0].score " << pods[0].score <<" pods[2].score " << pods[2].score << endl;
        Vector2D offset =checkpoints[pod.check_point_id] - pod.position;
      /*  if(offset.Magnitude()<3000.0f)
        {
            level = level+2;
            cerr<< " level " << level << endl;
            thrusts = thrusts1;
            angles = angles1;
        }*/
        //cerr<< " pod.angle  " << pod.angle << endl;
        for (int cangle = 0; cangle < angles.size(); ++cangle)
        {
            for (int cthrust = 0; cthrust < thrusts.size(); ++cthrust)
            {
                PodData p1(pod); 
                PodData p2(pods[1]);
                PodData o1(pods[2]);
                PodData o2(pods[3]);          
                float  mfit;        
                float t[3];
                        
              
              
                moveAltogether(p1, angles[cangle], thrusts[cthrust],
                        true, p2, o1, o2, mfit, false);

                max_fit_shielded = max(max_fit_shielded, mfit);
               
               //cerr<< "jestem after checked  " <<   " p1.score " <<  p1.score << " pods[0].score " <<pods[0].score <<endl;
                //cerr<< "p1.check_point_id after "<< p1.check_point_id << endl;
                //cerr<< "p1.position after " << p1.position.x << " " << p1.position.y << endl;
                //cerr<< " mfit " << mfit <<" max_fit "<< max_fit<< endl;
                //max_fit_shielded = max(max_fit_shielded, mfit);
                
                Vector2D target;
                target = checkpoints[p1.check_point_id];
               

                
                Vector2D offset = checkpoints[(p1.check_point_id-1)%nr_check_points] - p1.position;
                float dist = offset.Magnitude();
                offset.Normalize();
                //Vector2D dir = getDirection();            
                float angle1 = offset.getAngle();
                //abs(acos(offset.DotProduct(dir)));
                float diff_angle = p1.angle * PI/180.0f - angle1;
        
        //return dist + pod.speed.Magnitude() * angle * angle * 10.0f;

                cerr<< " dist " << dist<< endl;
                if(!false)
                {
                    dist+= p1.speed.Magnitude() * abs(diff_angle) * 50.0f;
                }
                cerr<< " dist after " << dist<< endl;

                int th = thrusts[cthrust];
                float fitness = PickThrustAngleHelper(p1, p2, o1, o2, th, 
                    target, angles, thrusts1, level);
                //cerr<< " getRelativeFitness(p1, thrust, o1, o2 ); "<< getRelativeFitness(p1, thrusts[cthrust] , o1, o2 ) <<endl;
                //cerr << " max_fit < fitness " << max_fit << " " << fitness << endl;
                //cerr<< " p1.getScore() " << p1.getScore() << endl;
                if (max_fit < fitness )
                {
                   // cerr<< " angles[cangle] " << angles[cangle] << endl;
                    max_fit = fitness;
                    best_pod = p1;
                    best_thrust = th; //thrusts[cthrust];
                }
                /*if (!passed_checkpoint && p.passed_checkpoint)
                {

                    passed_checkpoint = true;
                    min_dist = dist_eval;
                    best_pod = p;
                    best_thrust = th;//thrusts[cthrust];
                }
                else                     
                }*/
            }
        }
                
        //cerr << "max_fit_shielded " << max_fit_shielded << " max_fit " << max_fit<< endl;
        if(max_fit_shielded <= max_fit)
        {
            //cerr<<"max_fit "<< max_fit<<endl;
            Vector2D dir = best_pod.getDirection();
            Vector2D dd = pod.position + dir * 5000.0f;
        
            out_pod.out_x = (int)dd.x;
            out_pod.out_y = (int)dd.y;
            out_pod.sthrust = std::to_string((int)best_thrust);
        } else {
            Vector2D dir = pod.getDirection();
            Vector2D dd = pod.position + dir * 5000.0f;
        
            out_pod.out_x = (int)dd.x;
            out_pod.out_y = (int)dd.y;
            out_pod.sthrust = "SHIELD";
        }
    }

    //find best target and thrust by simulating real movements 
    //target is the winning opponent
    void PickThrustAngleInterceptor(OutputValues& out_pod, const PodData& pod, const Vector2D& target,
        const int& level)
    {

        vector<float> angles = { -18.0f ,-9.0f, 0.0f, 9.0f, 18.0f };
        vector<float> thrusts = { 5.0f, 55.0f, 100.0f };
        PodData best_pod, best_pod_shielded;
        float max_fitness  = VERY_SMALL_NUMBER;
        float max_fitness_shielded = VERY_SMALL_NUMBER;
        float best_thrust = MAX_THRUST;


        for (int cangle = 0; cangle < angles.size(); ++cangle)
        {
            for (int cthrust = 0; cthrust < thrusts.size(); ++cthrust)
            {
                PodData p1(pod); 
                PodData p2(pods[0]);
                PodData o1(pods[2]);
                PodData o2(pods[3]);          
                float m_fitness_shielded;
                //p1.addAngle(angles[cangle]);
                //p1.move(thrusts[cthrust]);        
                //cerr<< " getRelativeFitness(p2, thrust, o1, o2 ); "<< getRelativeFitness(p2, thrusts[cthrust] , o1, o2 ) <<endl;
                //p1.addAngle(angles[cangle]);
                
                moveAltogether(p1, angles[cangle], thrusts[cthrust],
                        false, p2, o1, o2, m_fitness_shielded);
                //cerr<< " getRelativeFitness(p2, thrust, o1, o2 ); after "<< getRelativeFitness(p2, thrusts[cthrust] , o1, o2 ) <<endl;        
                max_fitness_shielded = max(m_fitness_shielded, max_fitness_shielded);    
                
                int th = thrusts[cthrust];
                float fitness = PickThrustAngleInterceptorHelper(p1, p2, o1, o2,
                    th, target,
                    angles, thrusts, level);

                //PickThrustAngleInterceptorHelper(p1, p2, o1, o2,
                  //   th, target, angles, thrusts, level - 1);

                /*float fitness_chaser;// = getRelativeFitnessChaser(p1, p2, target, 
                 //   o1, o2);

                float scoreChaser =  p2.getScore();
                cerr << " scoreChaser " << scoreChaser << endl;
                //if(p0.passed_checkpoint)
                { 
                    scoreChaser-= p2.getDistanceEval(checkpoints[p2.check_point_id], thrusts[cthrust]);
                }
        
                //we assume that opponnents move with thrust 100
                float score2 = o1.getScore();
                //if(p2.passed_checkpoint)
                { 
                    score2 -= o1.getDistanceEval(checkpoints[o1.check_point_id]);
                }
                float score3 = o2.getScore();
                //if(p3.passed_checkpoint)
                { 
                    score3 -=  o2.getDistanceEval(checkpoints[o2.check_point_id]);
                }
                float opponent_score = max(score2, score3);
                cerr << " scoreChaser " << scoreChaser << " opponent_score "<< opponent_score << endl;
        //" opponent_score "<< opponent_score << endl;

                fitness =  scoreChaser - opponent_score;
                cerr << " scoreChaser - opponent_score " 
                    << scoreChaser - opponent_score << endl;
                //fitness = getRelativeFitness(p1, 0.0f, o1,  o2);
                fitness *= 100.0f;
                fitness -= p1.getDistanceEval(target);
                //fitness = getRelativeFitness(p1, th, o1, o2 ); */

                //cerr << " fitness  final " 
                //    << fitness << " p1.getDistanceEval(target) " << p1.getDistanceEval(target)<< endl;
            

                //cerr<< " max_fitness < fitness " << max_fitness << " " <<fitness<< endl;    
                if (max_fitness < fitness)
                {                   
                    
                    max_fitness = fitness;
                    best_pod = p1;
                    best_thrust = thrusts[cthrust];
                }
            }
        }

        if(max_fitness_shielded <= max_fitness)
        {
            Vector2D dir = best_pod.getDirection();
            Vector2D dd = pod.position + dir * 5000.0f;
        
            out_pod.out_x = (int)dd.x;
            out_pod.out_y = (int)dd.y;
            out_pod.sthrust = std::to_string((int)best_thrust);
        } else {
            Vector2D dir = pod.getDirection();
            Vector2D dd = pod.position + dir * 5000.0f;
        
            out_pod.out_x = (int)dd.x;
            out_pod.out_y = (int)dd.y;
            out_pod.sthrust = "SHIELD";
        }
        
    }

    //returns distance evaluation
    float PickThrustAngleInterceptorHelper(const PodData& pod, 
        const PodData& chaser, 
        const PodData& op2,
        const PodData& op3,
        int& pthrust,
        const Vector2D& target,
        const vector<float>& angles, 
        const vector<float>& thrusts, 
        int level)
    {

        //Vector2D offset = target - pod.position;

        //float dist = offset.Magnitude();
        if (level == 0)
        {
            //offset.Normalize();
            //Vector2D dir = pod.getDirection();
            //(cos(pod.angle * PI / 180.0f), sin(pod.angle * PI / 180.0f));

            //float angle = abs(acos(offset.DotProduct(dir)));
            //return dist + pod.speed.Magnitude() * angle * angle * 10.0f;
            float fitness = getRelativeFitnessChaser(chaser, pod , target, op2, op3 );
            //getRelativeFitness(chaser, pthrust, op2, op3 );
            //cerr<< " fitness in" << fitness << " chaser.position " << chaser.position.x 
            //    <<" "<< chaser.position.y<<endl;
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
                                
                int th = thrusts[cthrust];
                                                
                float fitness = getRelativeFitnessChaser(p2, p1, target, 
                    o1, o2);
                
                max_fit = max(max_fit, fitness);
                max_fit = max(max_fit, max_fit_shielded);
               
                
            }
        }
        
        if (level == SIMULATION_LEVEL)
        {
            //pthrust = picked_thrust;
            //cerr<<"pthrust "<< pthrust<<  " picked_thrust " <<picked_thrust<<endl;
        }
        return max_fit;

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

    bool moveAltogether(PodData& p1, const float angle, const float thrust,
             const bool passes_checkpoint, 
        PodData& p2, PodData& o1, PodData& o2, float& max_fit_shielded, const bool passed_checkpoint = false)
    {
        /*PodData p(pod);
        PodData ch(chaser);
        PodData opp2(op2);
        PodData opp3(op3);*/
                //cerr<< " p.score " << p.score << " opp2.score " << opp2.score<< endl;
                //float dist = p.move(thrusts[cthrust], angles[cangle]);
                
                //bool checkpoint_collide = false;

        float tt;
        float t[3];
        PodData op1(p1);
        PodData op2(p2);
        PodData oo1(o1);
        PodData oo2(o2);
                
        p1.addAngle(angle);
        bool col0 = p1.Collide( p2, t[0]);                
        bool col2 = p1.Collide( o1, t[1]);                
        bool col3 = p1.Collide( o2, t[2]);
        int collider = -1;

        max_fit_shielded = VERY_SMALL_NUMBER;
        if( col0 || col2 || col3 )
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
            //cerr<< "collider " << collider << endl;
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
            
                    
                    
        } else 

        {

            
            
            p1.move(thrust);
            

            if( !passes_checkpoint )
            {                
                //cerr<< " p2.position "<< p2.position.x << " " <<p2.position.y <<endl;        
            }           
            p2.move(100.0f);
            if( !passes_checkpoint )
            {                
               // cerr<< " p2.position  after"<< p2.position.x << " " <<p2.position.y <<endl;        
            }

            o1.move(100.0f);
            o2.move(100.0f);

            if( !passes_checkpoint )
            {                
                //cerr<< " getRelativeFitness(p2, thrust, o1, o2 ); after  "<< getRelativeFitness(p2, thrust , o1, o2 ) <<endl;        
            }
            
            if( !passes_checkpoint && !p2.passed_checkpoint )
            {
                //cerr<< "!passes_checkpoint " << endl;
                p2.passed_checkpoint = p2.Collide(checkpoints[p2.check_point_id], Vector2D::Zero(), tt);
                if(p2.passed_checkpoint)
                {
                    p2.check_point_id = (p2.check_point_id+1)% nr_check_points;
                    p2.score++;                 
                }
            }
            if( !o1.passed_checkpoint )
            {
                o1.passed_checkpoint = o1.Collide(checkpoints[o1.check_point_id], Vector2D::Zero(), tt);
                if(o1.passed_checkpoint && !passed_checkpoint)
                {
                    o1.check_point_id = (o1.check_point_id+1)% nr_check_points;
                    o1.score++;
                }
            }
            if( !o2.passed_checkpoint )
            {
                o2.passed_checkpoint = o2.Collide(checkpoints[o2.check_point_id], Vector2D::Zero(), tt);
                if(o2.passed_checkpoint )
                {
                    o2.check_point_id = (o2.check_point_id+1)% nr_check_points;
                    o2.score++;
                }
            }
            if( passes_checkpoint && !p1.passed_checkpoint )
            {                
                p1.passed_checkpoint = p1.Collide(checkpoints[p1.check_point_id], Vector2D::Zero(), tt);
                if(p1.passed_checkpoint)                
                {
                   // cerr<< "jestem checked  level " <<  << " p1.score " <<  p1.score <<endl;                    
                    p1.check_point_id = (p1.check_point_id+1)% nr_check_points;
                    p1.score++;                 
                }
            }
            

                
        }
        return col0 || col2 || col3;
    }

    Game() = default;

    float getRelativeFitnessChaser(const PodData& chaser, const PodData& p1, const Vector2D& target, 
        const PodData& p2, const PodData& p3)
    {
        float fitness = getRelativeFitness(chaser, 0.0f, p2,  p3, false);
        fitness *= 100.0f;
        fitness = -p1.getDistanceEval(target, true);
        
        //cerr << " scoreChaser " << scoreChaser << " opponent_score "<< opponent_score << endl;
        //" opponent_score "<< opponent_score << endl;

        return fitness;
    }

    float getRelativeFitness(const PodData& p0,const float& thrust, 
        const PodData& p2, const PodData& p3, const bool isInterceptor)
    {
        float scoreChaser = p0.getScore();
        //cerr << " scoreChaser " << scoreChaser << endl;
        //if(p0.passed_checkpoint)
        { 
            scoreChaser-= p0.getDistanceEval(checkpoints[p0.check_point_id], isInterceptor );
        }
        
        //we assume that opponnents move with thrust 100
        float score2 = p2.getScore();
        //if(p2.passed_checkpoint)
        { 
            score2 -= p2.getDistanceEval(checkpoints[p2.check_point_id], isInterceptor);
        }
        float score3 = p3.getScore();
        //if(p3.passed_checkpoint)
        { 
            score3 -=  p3.getDistanceEval(checkpoints[p3.check_point_id], isInterceptor);
        }
        float opponent_score = max(score2, score3);
        //cerr << " scoreChaser " << scoreChaser << " opponent_score "<< opponent_score << endl;
        //" opponent_score "<< opponent_score << endl;

        return scoreChaser - opponent_score;
    }

    void UpdatePod(const Vector2D& pos, const Vector2D& s, const int a, const int check_point_id,  const int id)
    {
        
        cerr<<  " id chuj: " << id << " check_point_id " <<check_point_id << endl;
        pods[id].UpdatePod(pos, s, a, check_point_id );
        //cerr<< " pods[id].score " << pods[id].score << " id " << id << " check_point_id " 
        if(id==0)
            cerr    << " scoree !!! "<< pods[id].score << endl;
        //        cerr << "angle pod " << a << endl;

    }

    Vector2D setTargetForInterceptor(const PodData& pod, int opp_id)
    {
        float t;
        cerr<< " jestem intercept "<< endl;
        Vector2D offset = pods[opp_id].position - pod.position;
        if( offset.Magnitude()< 6000.0f )
        {
            PodData opp(pods[opp_id]);
            for (int i = 0; i < 7; ++i)
            {
                opp.move(100);
            }
            cerr<< " opp.position "<< opp.position.x << " " << opp.position.y<<endl;
            cerr<< " pods[opp_id].position "<< pods[opp_id].position.x << " " << pods[opp_id].position.y<<endl;
            cerr<< " pod.position "<< pod.position.x << " " << pod.position.y<<endl;
            cerr<< " jestem intercept 111"<< endl;
            return opp.position;    
        } else {
            cerr<< " pods[opp_id].position "<< pods[opp_id].position.x << " " << pods[opp_id].position.y<<endl;
            return pods[opp_id].position;
        }

        //offset.Normalize();
        
        /*t = numeric_limits<float>::max();
        Vector2D soffset = pod.speed - pods[opp_id].speed;
        Vector2D poffset = pod.position - pods[opp_id].position;
        float a = soffset.DotProduct(soffset);
        float b = soffset.DotProduct(poffset) * 2.0f;
        float c = poffset.DotProduct(poffset) - 4.0f * (float)(POD_RADIUS * POD_RADIUS*1.2f);
        float rc = b*b -4 * a *c;
        
        rc = sqrt(rc);
        float x1 = (-b-rc)/(2.0f*a);
        float x2 = (-b+rc)/(2.0f*a);
        offset.Normalize();
        cerr << " b/a c/a x1 " << b/a << " "<<c/a <<" " <<x1 <<endl;*/
        //if(!pod.UCollide(pods[opp_id], t ))
        //{
        //    return pods[opp_id].position;
        //}
        //return  pod.position + offset*t;
        /*Vector2D dir = pods[1].getDirection();
        Vector2D offset_checkpoint1 = checkpoints[pods[opp_id].check_point_id] - pods[opp_id].position;
        Vector2D offset_checkpoint2 = checkpoints[pods[opp_id].check_point_id] - pods[1].position;
        bool first = offset_checkpoint2.Magnitude() > offset_checkpoint1.Magnitude();
        offset_checkpoint1.Normalize();
        first = (offset_checkpoint1.DotProduct(dir)) > 0.9f && first;
        cerr << " abs(offset_checkpoint1.DotProduct(dir)) " << (offset_checkpoint1.DotProduct(dir)) << endl;
        if (!first)
        {
            PodData opp(pods[opp_id]);
            for (int i = 0; i < OPPONENT_SIMULATION_LEVEL + 5; ++i)
            {
                opp.move(100, 0);
            }
            return opp.position;
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
        }*/
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

            
            /*PodData opp(pods[opp_id]);
            for (int i = 0; i < 7; ++i)
            {
                opp.move(100);
            }*/
            Vector2D target =  setTargetForInterceptor( pods[interceptor], opp_id);
            //opp.position;// setTargetForInterceptor(opp_id);
            PickThrustAngleInterceptor(out_pod2, pods[interceptor], target, OPPONENT_SIMULATION_LEVEL);
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

       /* if (pods[chaser].shouldShield(checkpoint_dir, pods[interceptor]))
        {
            out_pod1.sthrust = "SHIELD";
        }*/
        /*if (pods[chaser].shouldShield(checkpoint_dir, pods[2]))
        {
            out_pod1.sthrust = "SHIELD";
        }
        if (pods[chaser].shouldShield(checkpoint_dir, pods[3]))
        {
            out_pod1.sthrust = "SHIELD";
        }*/

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

      /*  if (pods[interceptor].shouldShield(checkpoint_dir, pods[opp_id]))
        {
            out_pod2.sthrust = "SHIELD";
        }
        if (pods[interceptor].shouldShield(pods[5 - opp_id]))
        {
            out_pod2.sthrust = "SHIELD";
        }*/



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

