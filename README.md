
## Model Description

`state` vector, which holds the current state of the vehicle, and it consists of the below six values:
 * x: x component of the vehicle current position
 * y: y component of the vehicle current position
 * psi: current heading (orientation) angle of the vehicle in radians measured anticlockwise from the global x-axis
 * v: current velocity of the vehicle
 * cte: cross track error approximated as the distance between the desired path and
 * the vehicle along the vehicle's y-axis (horizontal to the vehicle)
 * epsi: orientation angle error

`vars` vector which holds the current and future values of the state and
actuation variables as one long sequential vector, as shown below:
 
if the number of timesteps N = 10, then the "vars" vector will look like this:
 
vars[0], ..., vars[9] -> x<sub>1</sub>, ...,x<sub>10</sub>  

vars[10], ..., vars[19] -> y<sub>1</sub>, ...,y<sub>10</sub>  

vars[20], ..., vars[29] -> ψ<sub>1</sub>, ...,ψ<sub>10</sub>  

vars[30], ..., vars[39] -> v<sub>1</sub>, ...,v<sub>10</sub>  

vars[40], ..., vars[49] -> cte<sub>1</sub>, ...,cte<sub>10</sub>  

vars[50], ..., vars[59] -> epsi<sub>1</sub>, ...,epsi<sub>10</sub>  

vars[60], ..., vars[69] -> δ<sub>1</sub>, ...,δ<sub>10</sub>  

vars[70], ..., vars[79] -> a<sub>1</sub>, ...,a<sub>10</sub>  

`fg` vector, which holds the cost function as it's first value (fg[0]), and it holds each of the state variables initial value:

```C++
fg[1 + x_start] = vars[x_start];
fg[1 + y_start] = vars[y_start];
fg[1 + psi_start] = vars[psi_start];
fg[1 + v_start] = vars[v_start];
fg[1 + cte_start] = vars[cte_start];
fg[1 + epsi_start] = vars[epsi_start];
```

and each of the state variables subsequent values, based on the model equations (constraints):

```C++
for (int t = 1; t < N; t++) {
  ...
  ...
  ...
  fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
  fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
  fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
  fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
  fg[1 + cte_start + t] =
      cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
  fg[1 + epsi_start + t] =
      epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
}
```

 
---

The Ipopt solver uses the `fg` and `vars` vector, along with the defined boundaries for each of the variables and the constraints, to calculate the solution.

We extract from the solution only the next steering (δ) and acceleration (a) values, and return them to the simulator,
which are what we need for the next actuation, and discard the rest of the actuation values,
which are needed to compute the solution, but we don't need them to execute the next actuation,
also we extract all the future values for x and y, to visualize the predicted path that the vehicle will follow.

## Setting (N & dt)

I have chosen the values of N (number of timesteps) & dt (duration of timesteps) through trial and error, I've tried setting N to 20,
which works reasonably well, but it is very computationally intensive as it has to predict too many values into the future,
also as we predict more into the future, the path becomes more complicated, and a third degree polynomial will have trouble
following it accurately, I have tried setting it to 5, which seems not to have enough values to predict a correct path,
so it behaves erratically, thus I've settled for a value of 10.

for dt, I have tried 0.05, which works well but causes the vehicle to
vibrate quite a lot around the desired path, which seems to be because the actuation variables
are updated more often than they should be, and I've tried 0.4 but it doesn't accurately follow the desired path,
as the predicted path seems to have lower resolution than the desired path, so I've settled for 0.1 which worked quite well.

## Preprocessing waypoints

Initially the waypoints are shifted to the vehicle position, and rotated to the vehicle orientation, to transform them
from the global coordinates to the vehicle coordinates, which helps simplify the use of the `polyfit()` function, and 
consequentially the first 3 values of the current state (x, y, ψ) becomes zero, since we are using the vehicle coordinates
as our reference, which simplifies the calculation of cte and epsi.

## Managing Latency (100ms)

I've managed to solve the latency problem, by using the model equations to correct for the delayed initial state,
while setting dt in the equations to 0.1 (100ms), which outputs the state after 100ms, and then using that as the
new corrected initial state, here we note that the equations require (delta0) and (a0),
which are the current steering and acceleration values, so these were taken from the simulator in `main.cpp`, and passed
as additional parameters through the `solve()` function, and then used in the equations.

