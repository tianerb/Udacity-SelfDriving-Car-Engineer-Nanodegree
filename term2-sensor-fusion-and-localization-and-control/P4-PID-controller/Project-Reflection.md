
###  Reflection

#### Describe the effect each of the `P`, `I`, `D` components had in your implementation.

If I only use the `P` component, the car is able to drive for a while, but it will keep oscilating around center and will eventually fall off the road.

The `D` component can smooth the movement (reducing overshooting and oscilation) by considering the derivative of the `cte` value. But I had to tune it for a while. I originally set it too large so that the effect is too big and the car tends to turn too slowly and can't go back to center fast enough. However, the car still couldn't stick to the center. This is due to the systematic error/tendency.

The `I` component is designed to deal with systematic error by taking into account the total sum of `cte` over time. I tuned the parameter and the three parameters together have done a good job.

#### Describe how the final hyperparameters were chosen.
I guess a systematic way to tune the parameter would take too long, so I just manually tuned the three parameters. I started off adding each componet separately. I did a lot of test on the simulator eventually to find some combination of parameters that work.

I also tried to increase the `throttle` from 0.3 to 1.0. At 1.0 the car accelarates really fast and always fall off the road at some point with a speed over 90. However, a `throttle` value of 0.7 or 0.8 will let the car keep driving with some part of it off the road.

Some empirical facts I found:

`P` too large will cause larger oscilation and randomly off the road, too small will cause slower reaction at turns

`D` too large will cause slower reaction, too small will cause larger oscilation

`I` too large will cause the car react slowly at current turns (althogh it seems not very sentitive when increasing), too small will cause car off the road at the beginning due to systematic tendency
