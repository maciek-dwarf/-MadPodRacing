From wood to bronze:
Just adding boost if the distance to the checkpoint is big enought,
putting  thrust smaller scaled to angle and adding trust scale according to the distance
to checkpoint was good enought.

From bronze to silver/gold:

Added slowdawn before the checkpoint (some treshold) (scaled with distance).
Slowdawn after the checkpoint only if angle is big enought > 90.
Checking position betwen player and opponent and 
condition thrust value. 
The problem with that was that 
sometimes pod was drifting for no apparent reason.
I tried to adjust position of the checkpoint to avoid it.
Finally empirically i found adjustment :
checkpoint pos -= 3(current pos - previous pos);
This solution worked really well  and brought me directly to gold.

Gold:
1 version - second pod try to intercept follows winning opponent.
Boost is on the longest route betwen pods.
Given whole raceing path i precomputed parts where we want ot slowdawn.
Angle betwen this path (current checkpoint - previous checkpoint) and
(next checkpoint - current checkpoint) bigger then treshold.
Interceptor- work in progress.

Update:
Rank 1 in gold!!!!
Changed completely idea. Whole solution is based now on simulating movements of the pods.
Given rules we can compute position, speed and angle after 1 sec.
I also tried to simulate collision but it didnt work to well soo i switched
to basic shielding ( checking relative position of the pods and if they collide 
activate shield. I didn't include boost to simulation - its activated in the first turn.
I improved algorithm of the interceptor. He computes first checkpoint on the path of winning opponent's pod
which is closer to him that enemy pod. He goes to this checkpoint. If distance from pod and opponent is small enought
interceptor tries to cut opponent's path (by simulating few moves of the opponent).
We can determine winning opponent by tracking opponents pod checkpoints passed and  distance to current checkpoint.
Simulating moves is done by a recursive funtion. In each lvl of recursion we try few angles and trusts and simulate  
moves of all the pods. When we get to final level of recursion we return estimation of fitness of the pods.
Fitness of these 4 pods is a diffrence betwen score of our chasing pod and score of the winning opponent's pod.
Score is the BIG_NUMBER* nr_checkpoints_passed  - distance to next checkpoint.
We pick pod's movements to maximaze it's fitness.

Final remarks:
Unfortunetely simulation of the collision didnt work 2 well 
( i believe that by fixing this - a lot of debugging- i would get to legend).
I left it commented in the code.
Debugging  with cerr output - nightmare even on the GPU it's easier.
On the forum i read about genetic algorithm approach which apparently get's to legend but 
i haven't implemented it.
Was interesting and fun!!!!







 

