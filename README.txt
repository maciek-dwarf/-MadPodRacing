From wood to bronze:
Just adding boost if the distance to the checkpoint is big enought,
putting  thrust smaller scaled to angle and adding trust scale according to the distance
to checkpoint was good enought.
From bronze to silver/gold:
Added slowdawn before the checkpoint (some treshold) (scaled with distance).
Slowdawn after the checkpoint only if angle is big enought > 90.
Checking position betwen player and opponent and 
condition thrust value. 
The problem main problem just with that was that 
sometimes pod was drifting for no apparent reason.
I tried to adjust position of the checkpoint to avoid it.
Finally empirically i found adjustment :
checkpoint pos -= 3(current pos - previous pos);
This solution worked really well  and brought me directly to gold.

