#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace std;

/**
 * Auto-generated code below aims at helping you parse
 * the standard input according to the problem statement.
 **/
namespace {
    constexpr int SLOW_DAWN_TRESHOLD = 1800;
    constexpr int BOOST_ANGLE_TRESHOLD = 5;
    constexpr int BOOST_DISTANCE_TRESHOLD = 4000;
    constexpr int LONG_ROUTE_TRESHOLD = 4000;
    
    constexpr int ANGLE_TRESHOLD = 80;
    constexpr int MAX_THRUST = 100;
    constexpr int MIN_THRUST = 60;
    constexpr int MIN_PTHRUST = 30;
    constexpr int PREV_SLOW_DAWN_TRESHOLD = 1500;
    constexpr int COLLISION_TRESHOLD = 2500;
    constexpr int PLAYER_COLLISION_TRESHOLD = 2100;
}

class Game
{
    private:
        int in_pos_x;
        int in_pos_y;        
        int checkpoint_x;
        int checkpoint_y;
        int prev_checkpoint_x;
        int prev_checkpoint_y;
        int checkpoint_dist;
        int checkpoint_angle;
        int opponent_x;
        int opponent_y;
        float prev_dist;
        float route_dist;
        int speed_x;
        int speed_y;
        
    public:
    Game()
    {
        in_pos_x = 0;
        in_pos_y = 0;
        checkpoint_x = 0;
        checkpoint_y = 0;
        checkpoint_dist = 0;
        checkpoint_angle = 0;
        opponent_x = 0;
        opponent_y = 0;
        prev_checkpoint_x = 1000000000;
        prev_checkpoint_y = 1000000000;
        route_dist = std::numeric_limits<float>::max();        
    }

    void UpdateParams(const int& x,
                const int& y,
                const int& next_checkpoint_x,
                const int& next_checkpoint_y,
                const int& next_checkpoint_dist,
                const int& next_checkpoint_angle,        
                const int& opp_x,
                const int& opp_y)
    {
        speed_x = x - in_pos_x;
        speed_y = y - in_pos_y;
        in_pos_x = x;
        in_pos_y = y;
        bool changepod = false;
        if(checkpoint_x != next_checkpoint_x || checkpoint_y != next_checkpoint_y)
        {
            prev_checkpoint_x = checkpoint_x;
            prev_checkpoint_y = checkpoint_y;
            changepod = true;
        }
        float offset_x = in_pos_x - prev_checkpoint_x;
        float offset_y = in_pos_y - prev_checkpoint_y;
        prev_dist  = std::sqrt((float)(offset_x*offset_x + offset_y*offset_y));
        if(changepod)
        {
            route_dist = next_checkpoint_dist;
        }
        
        checkpoint_x = next_checkpoint_x;
        checkpoint_y = next_checkpoint_y;
        checkpoint_x -= 3*speed_x;
        checkpoint_y -= 3*speed_y;
        checkpoint_dist = next_checkpoint_dist;
        checkpoint_angle = next_checkpoint_angle;
        opponent_x = opp_x;
        opponent_y = opp_y;
    }

    void ComputeValues(int& out_x, int& out_y, std::string& sthrust, bool& boost)
    {
        int thrust;
        //checking angle
        if (checkpoint_angle > ANGLE_TRESHOLD || checkpoint_angle < -ANGLE_TRESHOLD)
        {
            thrust = (int)(1.0f - (float)(checkpoint_angle)/90.0f);
            thrust = std::clamp(thrust, MIN_THRUST, MAX_THRUST);            
        }            
        else{             
            thrust = MAX_THRUST;
        }

        //float thrust_scale = std::abs((float) checkpoint_angle / (float) ANGLE_TRESHOLD);
        //thrust -= thrust_scale*thrust;

        cerr << "distance " << checkpoint_dist << endl;
        cerr << "angle " << checkpoint_angle << endl;
        cerr << "route_dist " << route_dist << endl;
        
        
        //checking distance
        
        cerr << " prev_dist " << prev_dist << endl;
        int slow_dawn_treshold = max ( SLOW_DAWN_TRESHOLD, (int)(route_dist/4));
        cerr << "slow_dawn_treshold " << slow_dawn_treshold << endl;
        if(checkpoint_dist<slow_dawn_treshold)
        {
            float thrust_scale = std::abs((float)checkpoint_dist /(float)slow_dawn_treshold);
            
            thrust_scale = std::clamp(thrust_scale, 0.0f, 1.0f);
            
            
            thrust_scale = (float)MIN_THRUST + ((float)(MAX_THRUST-MIN_THRUST)* thrust_scale);
            thrust_scale /= (float)MAX_THRUST;
            thrust *= thrust_scale;
            //thrust = std::clamp(thrust, MIN_THRUST, MAX_THRUST);
            cerr << "  thrust_scale " << thrust_scale << endl;
            cerr << "  thrust " << thrust << endl;
        } else if(prev_dist <  PREV_SLOW_DAWN_TRESHOLD && route_dist < LONG_ROUTE_TRESHOLD 
        && std::abs(checkpoint_angle) > 90)
        {
            float thrust_scale = std::abs((float)prev_dist / (float) PREV_SLOW_DAWN_TRESHOLD);            
            thrust_scale = std::clamp(thrust_scale, 0.0f, 1.0f);
            
            thrust *= thrust_scale;            
            thrust = std::clamp(thrust, MIN_PTHRUST, MAX_THRUST);
            cerr << "  thrust prev dist " << thrust << endl;
        }
        //float offset_x = in_pos_x - prev_checkpoint_x;
        float offset_pl_opp_x =  in_pos_x - opponent_x;
        float offset_pl_opp_y =  in_pos_y - opponent_y;
        float opponent_player_dist = std::sqrt(offset_pl_opp_x*offset_pl_opp_x +
         offset_pl_opp_y*offset_pl_opp_y);
        float offset_pod_opp_x =  checkpoint_x - opponent_x;
        float offset_pod_opp_y =  checkpoint_y - opponent_y; 
        float opponent_pod_dist = std::sqrt(offset_pod_opp_x*offset_pod_opp_x +
         offset_pod_opp_y*offset_pod_opp_y);
        cerr << " opponent_player_dist " <<  opponent_player_dist<< endl;
        if(PLAYER_COLLISION_TRESHOLD > opponent_player_dist && std::abs(checkpoint_angle) < 90 )
        {
            thrust = MAX_THRUST;
        }
       // if(COLLISION_TRESHOLD > opponent_pod_dist && COLLISION_TRESHOLD >  checkpoint_dist)
        {
         //   thrust = 70;
        }
        //computing boost
        if(!boost)
        {
            boost =  std::abs(checkpoint_angle)  < BOOST_ANGLE_TRESHOLD && checkpoint_dist > BOOST_DISTANCE_TRESHOLD;
            if(boost)
            {
                cerr << "boost " << endl;
                sthrust = "BOOST";
            } else{
                cerr << " thruuustttt " << endl;
                sthrust = std::to_string(thrust);
            }
        } else {
            sthrust = std::to_string(thrust);
        }
        
        out_x = checkpoint_x;
        out_y = checkpoint_y;
        
    }

    
};

int main()
{
    bool boost = false;
    Game game;
    // game loop
    while (1) {
        int x;
        int y;
        int next_checkpoint_x; // x position of the next check point
        int next_checkpoint_y; // y position of the next check point
        int next_checkpoint_dist; // distance to the next checkpoint
        int next_checkpoint_angle; // angle between your pod orientation and the direction of the next checkpoint
        cin >> x >> y >> next_checkpoint_x >> next_checkpoint_y >> next_checkpoint_dist >> next_checkpoint_angle; cin.ignore();
        int opponent_x;
        int opponent_y;
        cin >> opponent_x >> opponent_y; cin.ignore();

        // Write an action using cout. DON'T FORGET THE "<< endl"
        // To debug: cerr << "Debug messages..." << endl;


        // You have to output the target position
        // followed by the power (0 <= thrust <= 100)
        // i.e.: "x y thrust"
 
        game.UpdateParams(x, y, next_checkpoint_x, next_checkpoint_y, next_checkpoint_dist, next_checkpoint_angle, opponent_x, opponent_y);

        int pos_x, pos_y;
        std::string thrust;                        
        game.ComputeValues(pos_x, pos_y, thrust, boost);
        cout << pos_x << " " << pos_y << " " << thrust << endl;        
        
    }
}