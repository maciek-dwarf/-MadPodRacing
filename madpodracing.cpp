#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

using namespace std;

/**
 * Auto-generated code below aims at helping you parse
 * the standard input according to the problem statement.
 **/
namespace {
    constexpr int SLOW_DAWN_TRESHOLD = 2000;
    constexpr int BOOST_ANGLE_TRESHOLD = 5;
    constexpr int BOOST_DISTANCE_TRESHOLD = 4000;
    constexpr int DISTANCE_TOSLOWDAWN_TRESHOLD = 2000;
    constexpr int ANGLE_TRESHOLD = 80;
}

class Game
{
    private:
        int in_pos_x;
        int in_pos_y;        
        int checkpoint_x;
        int checkpoint_y;
        int checkpoint_dist;
        int checkpoint_angle;
        int opponent_x;
        int opponent_y;
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
        in_pos_x = x;
        in_pos_y = y;
        checkpoint_x = next_checkpoint_x;
        checkpoint_y = next_checkpoint_y;
        checkpoint_dist = next_checkpoint_dist;
        checkpoint_angle = next_checkpoint_angle;
        opponent_x = opp_x;
        opponent_y = opp_y;
    }

    void ComputeValues(int& out_x, int& out_y, int& thrust, bool& boost)
    {
        
        //checking angle
        if (checkpoint_angle > ANGLE_TRESHOLD || checkpoint_angle < -ANGLE_TRESHOLD)
        {
            thrust = 30;
        }            
        else{
             
            thrust = 100;
        }

        float thrust_scale = std::abs((float) checkpoint_angle / (float) ANGLE_TRESHOLD);
        //thrust -= thrust_scale*thrust;

        cerr << "distance " << checkpoint_dist << endl;
        cerr << "angle " << checkpoint_angle << endl;
        
        //checking distance
        if(checkpoint_dist<SLOW_DAWN_TRESHOLD)
        {
            
            thrust_scale = std::abs((float)checkpoint_dist / (float) SLOW_DAWN_TRESHOLD);
            thrust_scale+=0.2;
            if(thrust_scale>1.0f)
            {
                thrust_scale = 1.0f;
            }
            cerr << "  thrust_scale " << thrust_scale << endl;
            thrust = (int)(thrust_scale  *(float)thrust);
            thrust = 50;
            cerr << "  thrust " << thrust << endl;
        }
       

        //computing boost
        boost = !boost && std::abs(checkpoint_angle)  < BOOST_ANGLE_TRESHOLD && checkpoint_dist > BOOST_DISTANCE_TRESHOLD;
        if(boost)
        {
          //  cerr << "boost " << endl;
        
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
 /*       int thrust = 100;
        //if(next_checkpoint_dist < DISTANCE_TOSLOWDAWN_TRESHOLD )
        if (next_checkpoint_angle > 90 || next_checkpoint_angle < -90)
        {
            thrust = 0;
        }            
        else{
             
            thrust = 100;
        }

        float thrust_scale = std::abs((float) next_checkpoint_angle / 90.0f);
        thrust -= thrust_scale*thrust;

        cerr << "distance " << next_checkpoint_dist << endl;
        cerr << "angle " << next_checkpoint_angle << endl;
        
        
        if(next_checkpoint_dist<SLOW_DAWN_TRESHOLD)
        {
            float thrust_scale = std::abs((float) next_checkpoint_dist / (float) SLOW_DAWN_TRESHOLD);
            thrust = (int)(thrust_scale * thrust_scale * (float)thrust);
        }
        else{
            thrust = 100;
        }
        bool use_boost = !boost && std::abs(next_checkpoint_angle)  < BOOST_ANGLE_TRESHOLD && next_checkpoint_dist > BOOST_DISTANCE_TRESHOLD;
        if(use_boost)
        {
            cerr << "boost " << endl;
            boost = true;
        }
        */
        game.UpdateParams(x, y, next_checkpoint_x, next_checkpoint_y, next_checkpoint_dist, next_checkpoint_angle, opponent_x, opponent_y);

        int pos_x, pos_y, thrust;

        bool tmp = boost;                 
        game.ComputeValues(pos_x, pos_y, thrust, tmp);
        if(!boost && tmp)
        {
            boost = true;
            cout << pos_x << " " << pos_y << " " << "BOOST" << endl;
        } else 
        {
            cout << pos_x << " " << pos_y << " " << thrust << endl;
        }
        
    }
}