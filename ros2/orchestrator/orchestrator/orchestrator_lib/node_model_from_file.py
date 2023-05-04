from __future__ import annotations

import base64
import json
from typing import cast, final, Dict, Optional, Tuple
from orchestrator.orchestrator_lib.node_model import Cause, Effect, NodeModel, ServiceCall, ServiceName, \
    StatusPublish, TimeSyncInfo, TimerInput
from deepdiff import DeepDiff
from rclpy.serialization import serialize_message, deserialize_message


@final
class ConfigFileNodeModel(NodeModel):

    def state_sequence_push(self, message):
        message_bytes = serialize_message(message)
        h = base64.b64encode(message_bytes).decode()

        if self.state_sequence is not None:
            expected = self.state_sequence.pop()
            expected_decoded = base64.b64decode(expected)
            expected_deserialized = deserialize_message(expected_decoded, type(message))
            if message != expected_deserialized:
                diff = DeepDiff(expected_deserialized, message)
                raise RuntimeError(f"State sequence diverged at node \"{self.get_name()}\" for object {message}. "
                                   f"Expected: {expected_deserialized}. Diff: {diff}")
        self.state_recording.append(h)

    def dump_state_sequence(self):
        with open('state_sequence_' + self.get_name() + '.json', 'w') as f:
            json.dump(self.state_recording, f)

    def __init__(self, node_config: dict, name, remappings: Dict[str, str],
                 state_sequence: Optional[list] = None) -> None:

        self.state_sequence = state_sequence
        if self.state_sequence is not None:
            self.state_sequence.reverse()
        self.state_recording = []

        # Mappings from internal to external name
        mappings: dict[str, str] = {}

        inputs = set()

        # Initialize mappings by identity for all known inputs and outputs from
        # node config.
        for callback in node_config["callbacks"]:
            trigger = callback["trigger"]

            if isinstance(trigger, str):
                mappings[trigger] = trigger
                inputs.add(trigger)
            elif "type" in trigger and trigger["type"] == "topic":
                trigger = cast(str, trigger["name"])
                mappings[trigger] = trigger
                inputs.add(trigger)
            elif "type" in trigger and trigger["type"] == "timer":
                mappings["clock"] = "clock"

            elif "type" in trigger and trigger["type"] == "approximate_time_sync":
                for topic in trigger["input_topics"]:
                    mappings[topic] = topic
                    inputs.add(topic)
            else:
                raise NotImplementedError(
                    f"Callback type {trigger['type']} not implemented")

            for output in callback["outputs"]:
                if output in inputs:
                    raise RuntimeError(
                        f"Topic {output} of node {name} defined as both input and output!")
                mappings[output] = output

            for service in callback.get("service_calls", []):
                mappings[service] = service

        for service in node_config.get("services", []):
            mappings[service] = service

        # Apply remappings from launch config
        for internal_name, external_name in remappings.items():
            if internal_name not in mappings:
                raise RuntimeError(
                    f"Remapping for \"{internal_name}\" to \"{external_name}\" given, but \"{internal_name}\" is not known at node {name}")
            if not mappings[internal_name] == internal_name:
                raise RuntimeError(f"Duplicate remapping for topic {internal_name}"
                                   f" of node {name}: First remapped to"
                                   f" {mappings[internal_name]}, then again"
                                   f" to {external_name}!")
            mappings[internal_name] = external_name

        super().__init__(name, [(internal, external)
                                for internal, external in mappings.items()])

        # Mapping from external topic input to external topic outputs
        # Second tuple element specifies if action modifies data provider state.
        self.effects: dict[Cause, Tuple[list[Effect], bool]] = {}

        def add_effect(trigger: Cause, outputs, service_calls, changes_dp_state):
            output_effects: list[Effect] = []
            for output in outputs:
                output_effects.append(self.internal_topic_pub(output))

            if len(output_effects) == 0:
                # This is intentionally before service call: callback with service call still needs status publish
                # to notify orchestrator about callback end, since service call is not intercepted
                output_effects.append(StatusPublish())

            for service_call in service_calls:
                output_effects.append(ServiceCall(
                    self.topic_name_from_internal(service_call)))

            self.effects[trigger] = (output_effects, changes_dp_state)

        self.approximate_time_sync_infos: list[TimeSyncInfo] = []

        for callback in node_config["callbacks"]:
            trigger = callback["trigger"]

            if isinstance(trigger, str):
                trigger = self.internal_topic_input(trigger)
                add_effect(trigger, callback.get("outputs", []),
                           callback.get("service_calls", []), callback.get("changes_dataprovider_state", False))
            elif trigger.get("type", None) == "topic" and "name" in trigger:
                topic_name = trigger["name"]
                trigger = self.internal_topic_input(topic_name)
                add_effect(trigger, callback.get("outputs", []),
                           callback.get("service_calls", []), callback.get("changes_dataprovider_state", False))
            elif trigger.get("type", None) == "timer" and "period" in trigger:
                period = trigger["period"]
                ti = TimerInput(int(period))
                if ti in self.effects:
                    raise RuntimeError(
                        f"Multiple timers with period {period} for node {name}")
                add_effect(ti, callback.get("outputs", []),  # trigger dict
                           callback.get("service_calls", []), callback.get("changes_dataprovider_state", False))
            elif trigger.get("type",
                             None) == "approximate_time_sync" and "input_topics" in trigger and "slop" in trigger and "queue_size" in trigger:
                input_topics = trigger["input_topics"]
                slop = trigger["slop"]
                queue = trigger["queue_size"]
                for t in input_topics:
                    add_effect(self.internal_topic_input(t), callback.get("outputs", []),
                               callback.get("service_calls", []), callback.get("changes_dataprovider_state", False))
                    # Additional status callback
                    self.effects[self.internal_topic_input(
                        t)][0].append(StatusPublish())
                self.approximate_time_sync_infos.append(
                    TimeSyncInfo(tuple(input_topics), slop, queue)
                )
            elif "type" in trigger:
                trigger_type = trigger["type"]
                raise NotImplementedError(
                    f"Callback type {trigger_type} not implemented")
            else:
                raise RuntimeError(f"Invalid trigger for callback {callback}")

        self.services: list[ServiceName] = []
        for service in node_config.get("services", []):
            self.services.append(self.topic_name_from_internal(service))

    def get_possible_inputs(self) -> list[Cause]:
        return list(self.effects.keys())

    def effects_for_input(self, input: Cause) -> list[Effect]:
        return self.effects[input][0]

    def input_modifies_dataprovider_state(self, input: Cause) -> bool:
        return self.effects[input][1]

    def get_provided_services(self) -> list[ServiceName]:
        return self.services

    def time_sync_info(self, topic_name: str) -> None | TimeSyncInfo:
        for tsi in self.approximate_time_sync_infos:
            if topic_name in tsi.input_topics:
                return tsi
        return None

    def time_sync_infos(self) -> list[TimeSyncInfo]:
        return self.approximate_time_sync_infos
