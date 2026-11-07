# Test flights

Each file here is a scripted flight that runs in the simulator. These flights test the simulator and individual ideas such as the landing. They are not the competition mission, and they must not run on the real drone.

There is one test so far, [`landing_test.py`](landing_test.py). It starts the run, climbs to 0.8 m, flies 1 m forward, and lands by itself. Its results are in section 7 of [`../../docs/ARCHITECTURE.md`](../../docs/ARCHITECTURE.md).

Run it from the top folder of the repository, using one of these two commands. The first uses the landing design from the plan. The second leaves out the push-down step, so that we can see how much time that step saves.

```bash
simulator/run_test_flight.sh landing_test.py --with-push-down
```

```bash
simulator/run_test_flight.sh landing_test.py --without-push-down
```

The test first waits for the simulator's start service, which only exists in the simulator. If the service doesn't appear, the test stops with an error before it sends the drone anything. That is what protects the real drone if someone runs this file there by accident.
