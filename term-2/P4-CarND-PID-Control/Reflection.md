## Describe the effect each of the P, I, D components had in your implementation.
 - The **P**roportional component had the biggest observable effect on the car's behavior. It causes the car to steer to the CTE. The output signal is proportional to the input signal.
 - The **I**ntegral component acts against any biases which prevents the controller from reaching the desired target by temporal integration of the CTE. This only played a minorrole as I could not identify any bias in the cars bebaviour.
 - The **D**ifferential component acts against overshooting the center line, which can also be called stabilization.

## Describe how the final hyperparameters were chosen.
 - I started with the hyperparameters shown in the „PID implementation“ lesson and iterated offer them.
 - As the hardware I am using is rather slow and one lap takes way over two minutes to complete I could not implement any automatic hyperparameter optimization like twiddle. I was not able to run tests without the emulator. The low hardware is also the reason I changed the throttle value to 0.1 everything higher just resulted in too many frame drops.