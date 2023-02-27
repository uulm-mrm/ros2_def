from typing import cast, final
from orchestrator.orchestrator_lib.node_model import Cause, Effect, NodeModel, ServiceCall, ServiceName, SimpleRemapRules, StatusPublish, TopicInput, TopicPublish, TimerInput


@final
class ConfigFileNodeModel(NodeModel):

    def __init__(self, node_config: dict, name, remappings) -> None:

        # Mappings from internal to external name
        mappings: dict[str, str] = {}

        # Initialize mappings by identity for all known inputs and outputs from
        # node config.
        for callback in node_config["callbacks"]:
            trigger = callback["trigger"]
            if isinstance(trigger, str):
                mappings[trigger] = trigger
            elif isinstance(trigger, dict):
                if trigger["type"] == "topic":
                    trigger = cast(str, trigger["name"])
                    mappings[trigger] = trigger
                elif trigger["type"] == "timer":
                    mappings["clock"] = "clock"
                else:
                    raise NotImplementedError(f"Callback type {trigger['type']} not implemented")

            for output in callback["outputs"]:
                if output in mappings:
                    raise RuntimeError(f"Topic {output} of node {name} defined as both input and output!")
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

        super().__init__(name, [(internal, external) for internal, external in mappings.items()])

        # Mapping from external topic input to external topic outputs
        self.effects: dict[Cause, list[Effect]] = {}
        for callback in node_config["callbacks"]:
            trigger = callback["trigger"]
            match trigger:
                case str():
                    trigger = self.internal_topic_input(trigger)
                case {"type": "topic", "name": topic_name}:
                    trigger = self.internal_topic_input(topic_name)
                case {"type": "timer", "period": period}:
                    ti = TimerInput(int(period))
                    if ti in self.effects:
                        raise RuntimeError(f"Multiple timers with period {period} for node {name}")
                    trigger = ti
                case {"type": trigger_type, **rest}:
                    raise NotImplementedError(f"Callback type {trigger_type} not implemented")
                case _:
                    raise RuntimeError(f"Invalid trigger for callback {callback}")

            output_effects: list[Effect] = []
            for output in callback.get("outputs", []):
                output_effects.append(self.internal_topic_pub(output))

            if len(output_effects) == 0:
                # This is intentionally before service call: callback with service call still needs status publish
                # to notify orchestrator about callback end, since service call is not intercepted
                output_effects.append(StatusPublish())

            for service_call in callback.get("service_calls", []):
                output_effects.append(ServiceCall(self.topic_name_from_internal(service_call)))

            self.effects[trigger] = output_effects

        self.services: list[ServiceName] = []
        for service in node_config.get("services", []):
            self.services.append(self.topic_name_from_internal(service))

    def get_possible_inputs(self) -> list[Cause]:
        return list(self.effects.keys())

    def effects_for_input(self, input: Cause) -> list[Effect]:
        return self.effects[input]

    def get_provided_services(self) -> list[ServiceName]:
        return self.services
