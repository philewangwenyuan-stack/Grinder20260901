"""
Unit Converter for SL-LinkA Protobuf Messages

This module provides automatic conversion between wire values (integers) and
real units (meters, degrees) based on custom options defined in the proto file.

Custom proto options:
  - unit (field 50001): Target unit string, e.g., "m", "deg", "deg/s"
  - scale (field 50002): Divisor for conversion (wire_value / scale = real_value)

Usage:
    from sl_link import get_real_value, set_wire_value, to_dict_real
    from sl_link.message_gen import sl_link_pb2 as pb

    # Get a field value in real units
    imu = pb.ImuData()
    imu.body_pitch = 4530  # wire value (45.30 degrees * 100)
    pitch_deg = get_real_value(imu, 'body_pitch')  # Returns 45.3

    # Set a field from real units
    set_wire_value(imu, 'body_pitch', 45.3)  # Sets wire value to 4530

    # Convert entire message to dict with real units
    real_dict = to_dict_real(imu)
"""

from typing import Any, Dict, Optional, Union
from google.protobuf.message import Message
from google.protobuf.descriptor import FieldDescriptor

# Import the generated proto module to access custom options
from ..message_gen import sl_link_pb2 as pb

# Option field numbers (must match proto definition)
UNIT_OPTION_NUMBER = 50001
SCALE_OPTION_NUMBER = 50002


def _get_scale(field: FieldDescriptor) -> Optional[float]:
    """Get the scale option from a field descriptor, if present."""
    options = field.GetOptions()
    # Try to get the scale extension
    for ext_field, ext_value in options.ListFields():
        if ext_field.number == SCALE_OPTION_NUMBER:
            return float(ext_value)
    return None


def _get_unit(field: FieldDescriptor) -> Optional[str]:
    """Get the unit option from a field descriptor, if present."""
    options = field.GetOptions()
    for ext_field, ext_value in options.ListFields():
        if ext_field.number == UNIT_OPTION_NUMBER:
            return str(ext_value)
    return None


def _is_numeric_field(field: FieldDescriptor) -> bool:
    """Check if a field is a numeric type (not enum, string, bytes, or message)."""
    return field.type in (
        FieldDescriptor.TYPE_INT32,
        FieldDescriptor.TYPE_INT64,
        FieldDescriptor.TYPE_UINT32,
        FieldDescriptor.TYPE_UINT64,
        FieldDescriptor.TYPE_SINT32,
        FieldDescriptor.TYPE_SINT64,
        FieldDescriptor.TYPE_FIXED32,
        FieldDescriptor.TYPE_FIXED64,
        FieldDescriptor.TYPE_SFIXED32,
        FieldDescriptor.TYPE_SFIXED64,
        FieldDescriptor.TYPE_FLOAT,
        FieldDescriptor.TYPE_DOUBLE,
    )


def get_real_value(message: Message, field_name: str) -> Union[float, int, Any]:
    """
    Get a field value converted to real units.
    
    If the field has a scale option, returns wire_value / scale.
    For enums, strings, bytes, and messages, returns the raw value.
    
    Args:
        message: A protobuf message instance
        field_name: Name of the field to get
        
    Returns:
        The field value in real units (float if scaled, original type otherwise)
        
    Example:
        >>> imu = pb.ImuData()
        >>> imu.body_pitch = 4530  # 45.30 degrees
        >>> get_real_value(imu, 'body_pitch')
        45.3
    """
    descriptor = message.DESCRIPTOR
    field = descriptor.fields_by_name.get(field_name)
    if field is None:
        raise ValueError(f"Field '{field_name}' not found in {descriptor.name}")
    
    raw_value = getattr(message, field_name)
    
    # Enums: return as-is (or convert to int if you prefer)
    if field.enum_type is not None:
        return raw_value
    
    # Non-numeric types: return as-is
    if not _is_numeric_field(field):
        return raw_value
    
    # Check for scale option
    scale = _get_scale(field)
    if scale is not None and scale != 0:
        return raw_value / scale
    
    return raw_value


def set_wire_value(message: Message, field_name: str, real_value: Union[float, int]) -> None:
    """
    Set a field from a real unit value to its wire representation.
    
    If the field has a scale option, sets wire_value = real_value * scale.
    For fields without scale, sets the value directly (with type conversion).
    
    Args:
        message: A protobuf message instance
        field_name: Name of the field to set
        real_value: The value in real units (e.g., meters, degrees)
        
    Example:
        >>> imu = pb.ImuData()
        >>> set_wire_value(imu, 'body_pitch', 45.3)  # Sets to 4530
        >>> imu.body_pitch
        4530
    """
    descriptor = message.DESCRIPTOR
    field = descriptor.fields_by_name.get(field_name)
    if field is None:
        raise ValueError(f"Field '{field_name}' not found in {descriptor.name}")
    
    # Check for scale option
    scale = _get_scale(field)
    if scale is not None and scale != 0:
        wire_value = real_value * scale
    else:
        wire_value = real_value
    
    # Convert to appropriate integer type if needed
    if field.type in (
        FieldDescriptor.TYPE_INT32, FieldDescriptor.TYPE_INT64,
        FieldDescriptor.TYPE_UINT32, FieldDescriptor.TYPE_UINT64,
        FieldDescriptor.TYPE_SINT32, FieldDescriptor.TYPE_SINT64,
        FieldDescriptor.TYPE_FIXED32, FieldDescriptor.TYPE_FIXED64,
        FieldDescriptor.TYPE_SFIXED32, FieldDescriptor.TYPE_SFIXED64,
    ):
        wire_value = int(round(wire_value))
    
    setattr(message, field_name, wire_value)


def get_field_info(message: Message, field_name: str) -> Dict[str, Any]:
    """
    Get metadata about a field, including unit and scale options.
    
    Args:
        message: A protobuf message instance
        field_name: Name of the field
        
    Returns:
        Dict with keys: 'name', 'type', 'unit', 'scale', 'is_enum'
    """
    descriptor = message.DESCRIPTOR
    field = descriptor.fields_by_name.get(field_name)
    if field is None:
        raise ValueError(f"Field '{field_name}' not found in {descriptor.name}")
    
    return {
        'name': field.name,
        'type': field.type,
        'type_name': FieldDescriptor.TYPE_TO_NAME.get(field.type, 'unknown'),
        'unit': _get_unit(field),
        'scale': _get_scale(field),
        'is_enum': field.enum_type is not None,
        'is_message': field.message_type is not None,
    }


def to_dict_real(message: Message, recursive: bool = True) -> Dict[str, Any]:
    """
    Convert a protobuf message to a dictionary with all values in real units.
    
    Fields with scale options are converted; enums and other fields are preserved.
    Nested messages are recursively converted if recursive=True.
    
    Args:
        message: A protobuf message instance
        recursive: If True, recursively convert nested messages
        
    Returns:
        Dict mapping field names to their real-unit values
        
    Example:
        >>> imu = pb.ImuData()
        >>> imu.body_pitch = 4530
        >>> imu.body_roll = 1234
        >>> to_dict_real(imu)
        {'body_pitch': 45.3, 'body_roll': 12.34, ...}
    """
    result = {}
    descriptor = message.DESCRIPTOR
    
    for field in descriptor.fields:
        field_name = field.name
        raw_value = getattr(message, field_name)
        
        # Handle repeated fields
        if field.label == FieldDescriptor.LABEL_REPEATED:
            if field.message_type is not None and recursive:
                # Repeated message field
                result[field_name] = [to_dict_real(item, recursive=True) for item in raw_value]
            else:
                # Repeated scalar field
                scale = _get_scale(field)
                if scale is not None and scale != 0:
                    result[field_name] = [v / scale for v in raw_value]
                else:
                    result[field_name] = list(raw_value)
            continue
        
        # Handle message fields
        if field.message_type is not None:
            if recursive:
                result[field_name] = to_dict_real(raw_value, recursive=True)
            else:
                result[field_name] = raw_value
            continue
        
        # Handle enum fields - return as-is (int value)
        if field.enum_type is not None:
            result[field_name] = raw_value
            continue
        
        # Handle scalar fields with potential conversion
        scale = _get_scale(field)
        if scale is not None and scale != 0 and _is_numeric_field(field):
            result[field_name] = raw_value / scale
        else:
            result[field_name] = raw_value
    
    return result


def from_dict_real(message: Message, data: Dict[str, Any], recursive: bool = True) -> None:
    """
    Populate a protobuf message from a dictionary with real-unit values.
    
    Fields with scale options are converted from real units to wire values.
    
    Args:
        message: A protobuf message instance to populate
        data: Dict mapping field names to real-unit values
        recursive: If True, recursively populate nested messages
        
    Example:
        >>> imu = pb.ImuData()
        >>> from_dict_real(imu, {'body_pitch': 45.3, 'body_roll': 12.34})
        >>> imu.body_pitch
        4530
    """
    descriptor = message.DESCRIPTOR
    
    for field_name, value in data.items():
        field = descriptor.fields_by_name.get(field_name)
        if field is None:
            continue  # Skip unknown fields
        
        # Handle repeated fields
        if field.label == FieldDescriptor.LABEL_REPEATED:
            repeated_field = getattr(message, field_name)
            if field.message_type is not None and recursive:
                # Repeated message field
                for item_data in value:
                    item = repeated_field.add()
                    from_dict_real(item, item_data, recursive=True)
            else:
                # Repeated scalar field
                scale = _get_scale(field)
                if scale is not None and scale != 0:
                    repeated_field.extend([int(round(v * scale)) for v in value])
                else:
                    repeated_field.extend(value)
            continue
        
        # Handle message fields
        if field.message_type is not None:
            if recursive and isinstance(value, dict):
                from_dict_real(getattr(message, field_name), value, recursive=True)
            continue
        
        # Handle enum fields - set directly
        if field.enum_type is not None:
            setattr(message, field_name, value)
            continue
        
        # Handle scalar fields with potential conversion
        set_wire_value(message, field_name, value)


def list_scaled_fields(message_type: type) -> Dict[str, Dict[str, Any]]:
    """
    List all fields in a message type that have unit/scale options.
    
    Args:
        message_type: A protobuf message class (not instance)
        
    Returns:
        Dict mapping field names to their unit/scale info
        
    Example:
        >>> list_scaled_fields(pb.ImuData)
        {'body_pitch': {'unit': 'deg', 'scale': 100.0}, ...}
    """
    result = {}
    descriptor = message_type.DESCRIPTOR
    
    for field in descriptor.fields:
        unit = _get_unit(field)
        scale = _get_scale(field)
        if unit is not None or scale is not None:
            result[field.name] = {
                'unit': unit,
                'scale': scale,
            }
    
    return result


# Convenience wrapper retained for generic scaled messages.
def message_to_real(message: Message) -> Dict[str, Any]:
    """Convert any protobuf message that carries scale options into real units."""
    return to_dict_real(message)
