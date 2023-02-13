from orchestrator_dummy_nodes.orchestrator_lib.node_model import Cause, Effect, NodeModel, SimpleRemapRules, StatusPublish, TopicInput, TopicPublish


class ConfigFileNodeModel(NodeModel):

    def __init__(self, node_config, name, remappings) -> None:

        # Mappings from internal to external name
        inputs: dict[str, str] = {}
        outputs: dict[str, str] = {}

        # Initialize mappings by identity for all known inputs and outputs from
        # node config.
        for callback in node_config["callbacks"]:
            trigger = callback["trigger"]
            if not isinstance(trigger, str):
                raise NotImplementedError("Timer triggers are not implemented yet!")
            inputs[trigger] = trigger
            for output in callback["outputs"]:
                if output in inputs:
                    raise RuntimeError(f"Topic {output} of node {name} defined as both input and output!")
                outputs[output] = output

        # Apply remappings from launch config
        for internal_name, external_name in remappings.items():
            if internal_name in inputs:
                if not inputs[internal_name] == internal_name:
                    raise RuntimeError(f"Duplicate remapping for topic {internal_name}"
                                       f" of node {name}: First remapped to"
                                       f" {inputs[internal_name]}, then again"
                                       f" to {external_name}!")
                inputs[internal_name] = external_name
            elif internal_name in outputs:
                if not outputs[internal_name] == internal_name:
                    raise RuntimeError(f"Duplicate remapping for output topic {internal_name}"
                                       f" of node {name}: First remapped to"
                                       f" {outputs[internal_name]}, then again"
                                       f" to {external_name}!")
                outputs[internal_name] = external_name

        input_list: SimpleRemapRules = [(internal, external) for internal, external in inputs.items()]
        output_list: SimpleRemapRules = [(internal, external) for internal, external in outputs.items()]

        super().__init__(name, input_list, output_list)

        # Mapping from external topic input to external topic outputs
        self.effects: dict[TopicInput, list[Effect]] = {}
        for callback in node_config["callbacks"]:
            trigger = callback["trigger"]
            if not isinstance(trigger, str):
                raise NotImplementedError("Timer triggers are not implemented yet!")
            outputs = callback["outputs"]
            # TODO: finalize output specification
            if not isinstance(outputs, list):
                raise NotImplementedError("Outputs other than topic names are not implemented")
            for output in outputs:
                if not isinstance(output, str):
                    raise NotImplementedError("Outputs other than topic names are not implemented")

            output_effects: list[Effect] = [self.internal_topic_pub(output) for output in outputs]
            if len(output_effects) == 0:
                output_effects.append(StatusPublish())

            self.effects[self.internal_topic_input(trigger)] = output_effects

    def get_possible_inputs(self) -> list[Cause]:
        return list(self.effects.keys())

    def effects_for_input(self, input: Cause) -> list[Effect]:
        if not isinstance(input, TopicInput):
            raise NotImplementedError("Other inputs than topics are not implemented for ConfigFileNodeModel.")
        return self.effects[input]
