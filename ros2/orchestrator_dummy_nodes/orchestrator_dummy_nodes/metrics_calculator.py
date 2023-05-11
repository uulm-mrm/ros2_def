import datetime

import aduulm_messages_python_api
from stonesoup.measures import Euclidean
from stonesoup.types.array import StateVector
from stonesoup.types.state import State
from stonesoup.metricgenerator.ospametric import OSPAMetric
import json

import matplotlib.pyplot as plt

from typing import List

ObjectState = aduulm_messages_python_api.ObjectState
ReferencePoint = aduulm_messages_python_api.ReferencePoint
Attr = aduulm_messages_python_api.Attr
StateType = aduulm_messages_python_api.StateType
initLogger = aduulm_messages_python_api.initLogger


def convertTimestep(json_timestep) -> List[State]:
    time = json_timestep["Time"]
    time_micros = time // 1000
    time_seconds = time_micros // 1000000
    timestamp = datetime.datetime.fromtimestamp(time_seconds)
    timestamp += datetime.timedelta(microseconds=(time_micros - time_seconds * 1000000))
    objects = []
    for object in json_timestep["Objects"]:
        model_id = object["state"]["model_id"]
        reference_point = object["reference_point"]
        state_array = object["state"]["mean"]
        x = ObjectState(
            model_id,
            state_array,
            [0] * len(state_array),
            [],
            ReferencePoint(reference_point)
        )
        state_vec = StateVector([x.getAttr(Attr.X), x.getAttr(Attr.Y)])
        objects.append(State(state_vec, timestamp=timestamp))
    return objects


def main():
    with open('/tmp/ttb.json') as f:
        tracks_file = json.load(f)
    with open('/tmp/gt.json') as f:
        gt_file = json.load(f)

    gt_states: List[State] = []
    tracks_states: List[State] = []

    for gt_timestep in gt_file:
        gt_states.extend(convertTimestep(gt_timestep))

    for track_timestep in tracks_file:
        tracks_states.extend(convertTimestep(track_timestep))

    gt_states = [gts for gts in gt_states if len([s for s in tracks_states if s.timestamp == gts.timestamp]) > 0]

    ospa_generator = OSPAMetric(c=1.0, p=1, measure=Euclidean())
    metric_result = ospa_generator.compute_over_time(tracks_states, gt_states)
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.plot([i.timestamp for i in metric_result.value], [i.value for i in metric_result.value])
    ax.set_ylabel("OSPA distance")
    # ax.tick_params(labelbottom=False)
    _ = ax.set_xlabel("Time")
    plt.show()


if __name__ == '__main__':
    main()
