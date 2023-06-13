import datetime
from dataclasses import dataclass

import aduulm_messages_python_api
import numpy as np
import shapely.geometry
import shapely.affinity
from stonesoup.measures import Euclidean
from stonesoup.types.array import StateVector
from stonesoup.types.state import State as SState
from stonesoup.metricgenerator.ospametric import OSPAMetric
import json

import matplotlib.pyplot as plt
import motmetrics as mm

from typing import List, Any, Dict

ObjectState = aduulm_messages_python_api.ObjectState
ReferencePoint = aduulm_messages_python_api.ReferencePoint
Attr = aduulm_messages_python_api.Attr
StateType = aduulm_messages_python_api.StateType
initLogger = aduulm_messages_python_api.initLogger


def convertTimestep(json_timestep) -> List[SState]:
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
        objects.append(SState(state_vec, timestamp=timestamp))
    return objects


def main():
    path = "/home/gja38/aduulm_sandbox_sil/data_for_eval"
    name = "_reconfig_nd_5.json"
    with open(path + "/ttb" + name) as f:
        tracks_file = json.load(f)
    with open(path + "/gt" + name) as f:
        gt_file = json.load(f)

    gt_states: List[SState] = []
    tracks_states: List[SState] = []

    for gt_timestep in gt_file:
        gt_states.extend(convertTimestep(gt_timestep))

    for track_timestep in tracks_file:
        tracks_states.extend(convertTimestep(track_timestep))

    gt_states = [gts for gts in gt_states if len([s for s in tracks_states if s.timestamp == gts.timestamp]) > 0]

    ospa_generator = OSPAMetric(c=2.0, p=1, measure=Euclidean())
    metric_result = ospa_generator.compute_over_time(tracks_states, gt_states)
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    timestamps = [i.timestamp for i in metric_result.value]

    values = [i.value for i in metric_result.value]
    data = np.column_stack(([t.timestamp() - 1e9 for t in timestamps], values))
    np.savetxt(name + ".csv", data, delimiter=",")
    ax.plot(timestamps, values)
    ax.set_ylabel("OSPA distance")
    # ax.tick_params(labelbottom=False)
    _ = ax.set_xlabel("Time")
    plt.show()


@dataclass
class State:
    state_vec: np.array
    track_id: object
    length: float
    width: float
    yaw: float

    def outline(self) -> shapely.geometry.polygon.Polygon:
        box = shapely.geometry.box(-self.width / 2, -self.length / 2, self.width / 2, self.length / 2)
        rotated_box = shapely.affinity.rotate(box, self.yaw)
        translated_box = shapely.affinity.translate(rotated_box, self.state_vec[0], self.state_vec[1])
        return translated_box


def convert_timestep_struct(json_timestep) -> (Any, List[State]):
    time = json_timestep["Time"]
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
        track_id = object["id"]
        state_vec = np.array([x.getAttr(Attr.X), x.getAttr(Attr.Y)])
        assert object["reference_point"] == 0
        objects.append(State(state_vec, track_id, x.getAttr(Attr.LENGTH), x.getAttr(Attr.WIDTH), x.getAttr(Attr.YAW)))

    return time, objects


def iou(s1: State, s2: State) -> float:
    # See mm.distances.iou_matrix() for docs
    o1 = s1.outline()
    o2 = s2.outline()

    i = o1.intersection(o2).area
    u = o1.union(o2).area

    iou_ = 1.0 - i / u
    if iou_ > 0.8:
        return float("nan")
    else:
        return iou_


def accumulate(name):
    with open('/home/gja38/aduulm_sandbox_sil/ttb_' + name + '.json') as f:
        tracks_file = json.load(f)
    with open('/home/gja38/aduulm_sandbox_sil/gt_' + name + '.json') as f:
        gt_file = json.load(f)

    gt_states: Dict[object, List[State]] = {}
    tracks_states: Dict[object, List[State]] = {}

    for track_timestep in tracks_file:
        t, s = convert_timestep_struct(track_timestep)
        tracks_states[t] = s

    for gt_timestep in gt_file:
        t, s = convert_timestep_struct(gt_timestep)
        if t in tracks_states:
            gt_states[t] = s
    acc = mm.MOTAccumulator()

    def norm(a: State, b: State):
        # return np.linalg.norm(a.state_vec - b.state_vec)
        return iou(a, b)

    for timestep, tracks in tracks_states.items():
        gt = gt_states[timestep]
        acc.update([s.track_id for s in gt],  # GT objects
                   [s.track_id for s in tracks],  # Object hypotheses
                   [  # For each object
                       [  # Distance to each hypothesis
                           norm(gt_s, h_s) for h_s in tracks
                       ] for gt_s in gt
                   ],
                   frameid=timestep
                   )
    return acc


def calculate_mot_metrics(names):
    accs = []
    for name in names:
        acc = accumulate(name)
        accs.append(acc)
    mh = mm.metrics.create()
    summary = mh.compute_many(accs, metrics=['num_frames', 'mota', 'motp'], names=names)
    print(summary.to_string())
    summary.to_csv("metrics.csv")


if __name__ == '__main__':
    # main()
    calculate_mot_metrics([*[f"nd_{i}" for i in range(3, 9)], *[f"o_{i}" for i in range(1, 3)]])
