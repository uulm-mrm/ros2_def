"""
This type stub file was generated by pyright.
"""


class ReconfigurationRequest_Request:
    ...


class ReconfigurationRequest_Response:
    """Message class 'ReconfigurationRequest_Response'."""

    new_launch_config_package: str
    "Message field 'new_launch_config_package'."

    new_launch_config_filename: str
    "Message field 'new_launch_config_filename'."


class ReconfigurationRequest:
    Request = ReconfigurationRequest_Request
    Response = ReconfigurationRequest_Response
