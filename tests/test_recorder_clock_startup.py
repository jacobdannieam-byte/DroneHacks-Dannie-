from examples.qualification_recorder import initial_clock_ready


def check(stamps):
    current = [0]
    elapsed = [0.]
    iterator = iter(stamps)
    def spin():
        elapsed[0] += .1
        current[0] = next(iterator, current[0])
    return initial_clock_ready(lambda: current[0], spin, lambda: elapsed[0], timeout_s=.8)


def test_waits_for_initialized_and_advancing_sim_clock():
    assert check((0, 0, 100_000_000, 100_000_000, 104_000_000))


def test_zero_stalled_and_rewound_clock_never_admit_recording():
    assert not check((0,))
    assert not check((100_000_000,))
    assert not check((100_000_000, 96_000_000))
