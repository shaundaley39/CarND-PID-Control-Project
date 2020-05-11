#!/usr/bin/env python

import os
import signal
import subprocess
from subprocess import Popen, PIPE


def run(speed, frames, p):
    simulator_process = subprocess.Popen("~/term2_sim_linux/term2_sim.x86_64", shell=True, preexec_fn=os.setsid, stdout=PIPE)
    process = Popen(["/home/ubuntu/Development/CarND-PID-Control-Project/build/pid"]+[str(speed), str(frames)]+[str(i) for i in p], stdout=PIPE)
    (output, err) = process.communicate()
    exit_code = process.wait()
    penalty = output.split('\n')[-2][len("Penalty: "):]
    print("applied penalty: "+penalty)
    os.killpg(simulator_process.pid, signal.SIGTERM)
    return float(penalty)

def twiddle(target_speed=20, p = [0.1, 0.0002, 0.2], r=10, tol=0.01):
    wp = [1 / i for i in p]
    dp = [0.1 * i for i in p]
    frames = 100 + (12000 / target_speed )
    f = [(r-1.0) / r, (r+1.0) / r]
    best_err = run(target_speed, frames, p)
    it = 0
    while sum([i*j for i,j in zip(wp,dp)]) > tol:
        print("Iteration {}, with p: {}, best error = {}".format(it, p, best_err))
        for i in range(len(p)):
            p[i] += dp[i]
            err = run(target_speed, frames, p)
            if err < best_err:
                best_err = err
                dp[i] *= f[1]
            else:
                p[i] -= 2 * dp[i]
                err = run(target_speed, frames, p)
                if err < best_err:
                    best_err = err
                    dp[i] *= f[1]
                else:
                    p[i] += dp[i]
                    dp[i] *= f[0]
        it += 1
    return p, best_err

target_speed = 20.0
d_speed = 10.0
p = [0.1, 0.0002, 0.2]
giving_up = 0

while(giving_up < 4):
  print("# Speed: {} mph".format(target_speed))
  p, best_err = twiddle(target_speed=target_speed, p=p)
  print("Speed: {}, attained a lowest penalty of {} with parameters {}".format(target_speed, best_err, p))
  if (best_err < 5000.0):
    target_speed += d_speed
  else:
    print("no good - dropping speed")
    d_speed = d_speed / 2.0
    target_speed -= d_speed
    giving_up += 1
