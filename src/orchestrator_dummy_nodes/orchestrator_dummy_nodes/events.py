from collections.abc import Mapping, Collection
from dataclasses import dataclass


class Cause:
    pass


@dataclass(frozen=True)
class TimerExpiry(Cause):
    start: float
    period: float
    node: str


@dataclass(frozen=True)
class TopicInput(Cause):
    input_topic: str
    node: str


class Effect:
    pass


@dataclass(frozen=True)
class TopicPublish(Effect):
    output_topic: str
    node: str

# Is it necessary to model the case in which a cause has variable effect, or variable number of specified effect(s)?
# (Might publish or not)
# Status message specifying which effects are/will be skipped (reordering effect/status might happen)?


CausalitySpecification = Mapping[Cause, Collection[Effect]]


node_1: CausalitySpecification = {TimerExpiry(0, 1, "node_1"): [TopicPublish("t1", "node_1"), TopicPublish("t2", "node_1")]}

intermediate_node_1: CausalitySpecification = {TopicInput("t1", "intermediate_node_1"): [TopicPublish("t", "intermediate_node_1")]}
intermediate_node_2: CausalitySpecification = {TopicInput("t2", "intermediate_node_2"): [TopicPublish("t", "intermediate_node_2")]}

node_2: CausalitySpecification = {TopicInput("t", "node_2"): [TopicPublish("out_1", "node_2")]}
node_3: CausalitySpecification = {TopicInput("t", "node_3"): [TopicPublish("out_2", "node_3")]}

timer_node: CausalitySpecification = {TimerExpiry(2.5, 2, "timer_node"): [TopicPublish("timer_out", "timer_node")]}


def time_until_next_execution(timer: TimerExpiry, current_time: float) -> float:
    if timer.start > current_time:
        return timer.start - current_time
    r = timer.period - ((current_time - timer.start) % timer.period)
    assert r > 0
    return r


def next_timer_after(timers: Collection[TimerExpiry], current_time: float):
    next_t = next(iter(timers))
    time_until_next = time_until_next_execution(next_t, current_time)
    for timer in timers:
        if time_until_next_execution(timer, current_time) < time_until_next:
            next_t = timer
            time_until_next = time_until_next_execution(timer, current_time)
    assert time_until_next > 0
    return next_t


def main():

    timers: list[tuple[TimerExpiry, Collection[Effect]]] = []
    nodes: list[CausalitySpecification] = [node_1, node_2, node_3, intermediate_node_1, intermediate_node_2, timer_node]

    for n in nodes:
        for cause, effects in n.items():
            if isinstance(cause, TimerExpiry):
                timers.append((cause, effects))

    queue: list[Cause | Effect] = []
    first_timer = next_timer_after([t for t, _ in timers], -0.00001)
    time = first_timer.start
    queue.append(first_timer)

    for _ in range(10):
        print("Time", time)
        event = queue[0]
        if isinstance(event, Cause):
            if isinstance(event, TimerExpiry):
                print("Timer expiry expected:", event)
                print("Publishing /clock to node to call timer")

                # TODO: We need node info here -> associate node and effects with cause

                print("Adding expected effects to queue")
            elif isinstance(event, TopicInput):
                print("Topic input expected. Releasing buffered message for node.")
        else:
            # TODO: Handle effects: Add topic publish and corresponding input causes to queue
            pass


if __name__ == '__main__':
    main()
